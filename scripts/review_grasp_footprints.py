"""Produce a reproducible offline cable-end footprint review, not a grasp result."""

import argparse
import json
import sys
from itertools import product
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from ffc.grasp_footprint import EndGeometry, rectangle_half_bounds, review  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    cable = json.loads((ROOT / "config/pi-zero-task.json").read_text())["cable"]
    end = EndGeometry(
        cable["mini_width_mm"], cable["assumed_exposed_length_mm"], cable["assumed_support_length_mm"]
    )
    half = rectangle_half_bounds(6, 3, 10)
    cases = []
    for name, setback, bounds in [
        ("Contacts", 2.5, half),
        ("Stiffener attempt", 5, half),
        ("Body pinch", 10, half),
        ("Body suction", 12, (3.5, 3.5)),
    ]:
        cases.append({"name": name, "setback_mm": setback, **review(end, 0, setback, bounds, 0.5, 1)})
    sweep = []
    # Sensitivity ranges are design hypotheses, not measured tolerances.
    for contacts, stiffener, error, yaw, setback in product(
        [3, 4, 5], [5, 6, 8], [0.25, 0.5, 1], [0, 5, 10], [5, 8, 10, 12]
    ):
        geometry = EndGeometry(end.width_mm, contacts, stiffener)
        sweep.append(
            {
                "contacts_mm": contacts,
                "stiffener_mm": stiffener,
                "placement_error_mm": error,
                "yaw_error_deg": yaw,
                "setback_mm": setback,
                **review(geometry, 0, setback, rectangle_half_bounds(6, 3, yaw), error, 1),
            }
        )
    report = {
        "scope": "Offline planar footprint review; no runtime control or physics",
        "cable_config": cable,
        "assumptions": {
            "pad_width_length_mm": [6, 3],
            "cup_compressed_diameter_mm": 7,
            "placement_error_each_axis_mm": 0.5,
            "yaw_error_deg": 10,
            "exclusion_margin_mm": 1,
            "uncertainties_are_measured": False,
        },
        "cases": cases,
        "sensitivity_cases": sweep,
        "runtime_motion_enabled": False,
        "limitations": [
            "Flat terminal only; no cable curvature or roll",
            "Full contact-length strip excluded on both faces",
            "Cup evaluated on body; no seal or vacuum physics",
            "No swept tool/desk/fixture or connector collision check",
            "No force, damage, buckling or successful handoff claim",
        ],
    }
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    (args.output / "source.py").write_bytes(Path(__file__).read_bytes())
    (args.output / "grasp_footprint.py").write_bytes((ROOT / "src/ffc/grasp_footprint.py").read_bytes())
    fig, axes = plt.subplots(1, 4, figsize=(15, 7), facecolor="#f6f4ef")
    for ax, case in zip(axes, cases, strict=True):
        ax.set_facecolor("#f6f4ef")
        ax.add_patch(Rectangle((-5.75, 0), 11.5, 21, color="#363e47"))
        # Bottom-face contact bands. Dashed line locates opposite-face stiffener end.
        for i in range(cable["mini_contacts"]):
            ax.add_patch(Rectangle((-5.25 + i * 0.5 - 0.15, 0), 0.3, 4, color="#d6b457"))
        ax.axhspan(0, 5, color="#cc4545", alpha=0.15)
        ax.plot([-5.75, 5.75], [6, 6], color="#49a9e6", ls="--", lw=2)
        y = case["setback_mm"]
        good = case["footprint_clear_under_assumptions"]
        color = "#14856c" if good else "#c44343"
        if case["name"] == "Body suction":
            patch = Circle((0, y), 3.5, fill=False, ec=color, lw=3)
            hx, hy = 4, 4
        else:
            patch = Rectangle((-3, y - 1.5), 6, 3, fill=False, ec=color, lw=3)
            hx, hy = half[0] + 0.5, half[1] + 0.5
        ax.add_patch(patch)
        ax.add_patch(Rectangle((-hx, y - hy), 2 * hx, 2 * hy, fill=False, ec=color, ls=":", lw=2))
        ax.set_title(case["name"] + f"\n{y:g} mm setback", fontsize=13, loc="left")
        ax.set(xlim=(-7, 7), ylim=(21, -1), aspect="equal", xticks=[], yticks=[0, 4, 6, 10, 15, 20])
        ax.set_xlabel("Footprint clears*" if good else "Rejected: contacts", color=color)
        for spine in ax.spines.values():
            spine.set_visible(False)
    axes[0].set_ylabel("Distance back from leading edge (mm)")
    fig.suptitle("Where can we touch the cable?", x=0.06, ha="left", fontsize=24)
    fig.text(
        0.06,
        0.89,
        "Contact-side view • Mini 22-pin end • Assumed dimensions, not measured geometry",
        fontsize=12,
    )
    fig.text(
        0.06,
        0.075,
        "Gold: exposed contacts     Blue dashed: stiffener ends on opposite face\n"
        "Dotted: footprint envelope with ±0.5 mm placement error; pads also include ±10° yaw\n"
        "*Planar clearance only. Pinch checks both faces. No seal, damage, handoff or insertion proof.",
        fontsize=11,
        linespacing=1.6,
    )
    fig.subplots_adjust(top=0.81, bottom=0.19, left=0.06, right=0.97, wspace=0.3)
    fig.savefig(args.output / "footprints.png", dpi=160)
    fig.savefig(args.output / "footprints.pdf")
    print(json.dumps({"cases": cases, "sensitivity_count": len(sweep)}, indent=2))


if __name__ == "__main__":
    main()
