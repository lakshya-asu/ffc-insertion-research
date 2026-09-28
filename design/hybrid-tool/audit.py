"""Check authored solid interferences at sampled mechanism poses (not certification)."""

import argparse
import json
from pathlib import Path

import cadquery as cq


def audit(directory):
    r = json.loads((directory / "assembly.json").read_text())
    items = []
    for p in r["parts"]:
        if "actuator" in p["name"] or p["kind"] == "envelope":
            continue
        shape = cq.importers.importStep(str(directory / "parts" / (p["name"] + ".step"))).val()
        items.append((p, shape))
    states = [("stowed", 0, 0, 0), ("lifted", 0, 0, 35)]
    states += [("lower_parked", 0, e, 35) for e in (3, 6, 9, 12, 15, 18)]
    states += [("deploy", d, 18, 35) for d in (3, 6, 9, 12, 15, 18)]
    states += [("close", 18, e, 35) for e in (16, 14, 12, 10, 8)]
    hits = []
    floor = []
    for label, d, e, lift in states:
        shapes = []
        for p, s in items:
            g = p["group"]
            delta = (0, 0, 0)
            if g == "carriage":
                delta = (d - 18, 0, 0)
            elif g == "jaw":
                delta = (d - 18, 0, 8 - e)
            elif g == "deploy_rod":
                delta = (d - 18, 0, 0)
            s = s.translate(delta)
            b = s.BoundingBox()
            if b.zmin + lift + 1.14 < -1e-5:
                floor.append(dict(phase=label, part=p["name"], z_mm=b.zmin + lift + 1.14))
            shapes.append((p, s, b))
        for i, (p, s, b) in enumerate(shapes):
            for q, t, c in shapes[i + 1 :]:
                if p["group"] == q["group"]:
                    continue
                # Bounding-box broad phase; face contact at zero volume is allowed.
                if any(
                    hi <= lo + 1e-5
                    for hi, lo in [
                        (min(b.xmax, c.xmax), max(b.xmin, c.xmin)),
                        (min(b.ymax, c.ymax), max(b.ymin, c.ymin)),
                        (min(b.zmax, c.zmax), max(b.zmin, c.zmin)),
                    ]
                ):
                    continue
                common = s.intersect(t)
                v = common.Volume() if common.Solids() else 0
                if v > 0.01:
                    hits.append(
                        dict(
                            phase=label,
                            deploy_mm=d,
                            extension_mm=e,
                            parts=[p["name"], q["name"]],
                            volume_mm3=v,
                        )
                    )
    result = {
        "states_checked": len(states),
        "interference_volume_threshold_mm3": 0.01,
        "floor_penetration_threshold_mm": 0.00001,
        "floor_intrusions": floor,
        "solid_interferences": hits,
        "excluded": [
            "manufacturer actuator shells",
            "sensor space reservation",
            "same-motion-group pairs",
            "cable",
            "fasteners not yet modeled",
        ],
        "scope": "Sampled authored solids only, not full collision or manufacturing qualification",
    }
    (directory / "audit.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("directory", type=Path)
    audit(p.parse_args().directory)
