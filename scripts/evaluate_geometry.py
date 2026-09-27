"""Run reproducible endpoint checks and render the exploratory workcell."""

# ruff: noqa: E402 -- EGL must be selected before importing MuJoCo.

from __future__ import annotations

import hashlib
import json
import os
import platform
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from ffc.workcell import build_model, load_config, solve_target, targets


def main() -> None:
    """Write actual IK results and a clearly labeled static scene preview."""
    output = ROOT / "outputs/e001"
    output.mkdir(parents=True, exist_ok=True)
    config = load_config()
    source = json.loads((ROOT / "config/fr3-source.json").read_text())
    results = {
        "experiment": "E001",
        "kind": "static_endpoint_geometry_only",
        "seed": config["seed"],
        "python": platform.python_version(),
        "mujoco": mujoco.__version__,
        "numpy": np.__version__,
        "model_revision": source["revision"],
        "config": config,
        "config_sha256": hashlib.sha256((ROOT / "config/workcell.json").read_bytes()).hexdigest(),
        "layouts": [],
    }
    for height in (config["fixture_height_m"], 0.005):
        scenario = {**config, "fixture_height_m": height}
        model, xml = build_model(scenario)
        records = [
            solve_target(model, target, scenario, config["seed"] + index)
            for index, target in enumerate(targets(scenario))
        ]
        results["layouts"].append({"fixture_height_m": height, "targets": records})
        count = sum(record["feasible_endpoint"] for record in records)
        print(f"Fixture {height * 1000:.0f} mm: {count}/{len(records)} feasible endpoints")
        for record in records:
            if not record["feasible_endpoint"]:
                print(
                    f"  FAILED {record['target']}: position={record['position_error_m']:.6f} m, "
                    f"rotation={record['orientation_error_rad']:.5f} rad, contacts={record['contacts']}"
                )
        if height == config["fixture_height_m"]:
            (output / "workcell.xml").write_text(xml)
            data = mujoco.MjData(model)
            data.qpos[:] = records[-1]["qpos"]
            mujoco.mj_forward(model, data)
            camera = mujoco.MjvCamera()
            camera.lookat[:] = [0.32, 0.0, 0.24]
            camera.distance, camera.azimuth, camera.elevation = 1.45, 135, -27
            options = mujoco.MjvOption()
            options.geomgroup[3] = 0  # Hide collision meshes while retaining collision computation.
            with mujoco.Renderer(model, height=1000, width=1600) as renderer:
                renderer.update_scene(data, camera=camera, scene_option=options)
                image = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, 1600, 84), fill=(17, 23, 29))
            draw.text((26, 20), "FR3 / FFC WORKCELL - GEOMETRY STUDY", fill="white", font_size=26)
            draw.text(
                (26, 53),
                "Static cable and tool proxies. No grasp, suction, or insertion physics.",
                fill=(171, 196, 212),
                font_size=18,
            )
            image.save(output / "workcell.png")
    (output / "results.json").write_text(json.dumps(results, indent=2) + "\n")


if __name__ == "__main__":
    main()
