"""Offline empty-socket collision diagnostic; no actor or robot motion."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--steps", type=int, default=200)
a = parser.parse_args()
if a.output.exists():
    raise FileExistsError(a.output)
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "extra_args": ["--allow-root"]})
try:
    import omni.usd
    from isaacsim.core.api import World
    from pxr import UsdGeom

    from ffc.isaac_contacts import ContactMonitor
    from ffc.socket_reference import build_socket

    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    cfg = json.loads((ROOT / "config/connectors/zero-reference-contact-v1.json").read_text())
    evidence = json.loads((ROOT / "config/connectors/pi-socket-evidence-v1.json").read_text())
    build_socket(stage, cfg, evidence, 0.055)
    world = World(physics_dt=0.000125, stage_units_in_meters=1)
    world.get_physics_context().set_solver_type("PGS")
    monitor = ContactMonitor(stage, 0.000125)
    world.reset()
    records = []
    for i in range(a.steps):
        monitor.begin_step(i)
        world.step(render=False)
        records.extend(monitor.current)
    a.output.write_text(json.dumps(records, indent=2))
    print(
        "DIAGNOSTIC",
        json.dumps(sorted(records, key=lambda r: r["sum_contact_force_magnitudes_n"], reverse=True)[:10]),
    )
    monitor.close()
    if any(r["sum_contact_force_magnitudes_n"] > 1e-6 for r in records):
        raise RuntimeError("Empty socket has unintended contact forces")
finally:
    app.close()
