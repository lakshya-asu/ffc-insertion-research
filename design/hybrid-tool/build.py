"""Build the two-axis hybrid-tool CAD concept in mm using CadQuery 2.6.1.

Manufacturer PQ12 STEP is an input, never synthesized. Output is a fit-review
assembly, not released manufacturing CAD. Run in a separate CAD environment.
"""

import argparse
import hashlib
import json
from pathlib import Path

import cadquery as cq


def box(size, center):
    return cq.Workplane("XY").box(*size).translate(center).val()


def cylinder(radius, length, base, axis):
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*base), cq.Vector(*axis))


def drilled_block(size, center, holes):
    solid = box(size, center)
    for radius, length, base, axis in holes:
        solid = solid.cut(cylinder(radius, length, base, axis))
    return solid


def build(source, output):
    output.mkdir(parents=True, exist_ok=False)
    imported = cq.importers.importStep(str(source))
    shells = imported.shells().vals()
    if len(shells) != 7:
        raise ValueError("Manufacturer assembly changed: inspect shell partition before proceeding")
    body = cq.Compound.makeCompound(shells[:5])
    shaft = cq.Compound.makeCompound(shells[5:])
    parts = []
    colors = {
        "printed": (0.10, 0.48, 0.42),
        "metal": (0.65, 0.69, 0.72),
        "purchased": (0.22, 0.25, 0.28),
        "soft": (0.13, 0.15, 0.17),
        "envelope": (0.88, 0.55, 0.20),
    }

    def add(name, shape, group="fixed", kind="printed", note="Custom concept geometry"):
        if shape.isNull() or not shape.isValid():
            raise ValueError(f"Invalid CAD: {name}")
        parts.append(dict(name=name, shape=shape, group=group, kind=kind, note=note))

    # Coordinates: x = deployment, y = ribbon length (tip +y), z = clamp normal.
    # Stowed pose: x=-18, clamp extension=0. Closed pose: x=0, extension=8 mm.
    # The 42.013 mm pivot spacing is extracted from cylindrical faces in vendor CAD.
    def orient_deploy(shape):
        return shape.rotate((0, 0, 0), (0, 1, 0), -90).translate((-63.013, -52.5, 102.5))

    add(
        "deploy_actuator_body",
        orient_deploy(body),
        kind="purchased",
        note="Manufacturer STEP; PQ12 family, electrical variant not encoded",
    )
    add(
        "deploy_actuator_shaft",
        orient_deploy(shaft).translate((18, 0, 0)),
        "deploy_rod",
        "purchased",
        "Manufacturer STEP; 18 mm extension at closed reference",
    )
    add(
        "clamp_actuator_body",
        body.translate((-5.5, -52.5, 70.013)),
        "carriage",
        "purchased",
        "Manufacturer STEP",
    )
    add(
        "clamp_actuator_shaft",
        shaft.translate((-5.5, -52.5, 62.013)),
        "jaw",
        "purchased",
        "Manufacturer STEP; retracts to close",
    )

    # Fixed frame: commercial metal stock keeps long-span stiffness out of prints.
    add(
        "backbone_plate",
        box((114, 6, 78), (-21, -91, 99)),
        kind="metal",
        note="Custom metal stock; bench fixture interface, FR3 bolt pattern unresolved",
    )
    add("overhead_spine", box((14, 100, 6), (28, -44, 135)), kind="metal")
    add("front_standoff", box((8, 12, 126), (28, 0, 69)), kind="metal")
    add("shoe_bridge", box((34, 12, 5), (13, 0, 8.5)), kind="metal")
    # Printed shoe seats standard cups; hose passages and sealing are external.
    shoe = drilled_block((14, 16, 6), (0, 0, 3), [(1.65, 8, (0, y, -1), (0, 0, 1)) for y in (-2.5, 2.5)])
    add(
        "vacuum_shoe",
        shoe,
        note=(
            "Print fit prototype; two M3 clearance bores. "
            "Thread retention and airtight fittings not qualified"
        ),
    )
    for i, y in enumerate((-2.5, 2.5)):
        add(
            f"cup_{i}",
            cylinder(1.85, 1, (0, y, -1), (0, 0, 1)),
            kind="soft",
            note="SUF 3 maximum-diameter envelope, not manufacturer CAD or lip deformation",
        )
        add(
            f"cup_stem_{i}",
            cylinder(1.5, 5.5, (0, y, 0), (0, 0, 1)),
            kind="metal",
            note="Connection envelope only",
        )

    # Guided lateral deployment. Rods are purchasable stock, bearings not selected.
    for i, y in enumerate((-65, -25)):
        add(
            f"deploy_guide_{i}",
            cylinder(2, 70, (-42, y, 90), (1, 0, 0)),
            kind="metal",
            note="4 mm metal shaft envelope; exact rail/bearing supplier open",
        )
        bush = cylinder(4.5, 14, (-7, y, 90), (1, 0, 0)).cut(cylinder(2.1, 16, (-8, y, 90), (1, 0, 0)))
        add(
            f"deploy_bearing_{i}",
            bush,
            "carriage",
            "purchased",
            "Bearing space reservation; running fit not specified",
        )
        for j, x in enumerate((-42, 28)):
            holder = drilled_block((8, 10, 18), (x, y, 90), [(2.05, 10, (x - 5, y, 90), (1, 0, 0))])
            add(f"guide_holder_{i}_{j}", holder)
    for j, x in enumerate((-42, 28)):
        add(f"guide_end_bridge_{j}", box((8, 68, 6), (x, -57, 78)), kind="metal")
    carriage = box((28, 48, 5), (0, -45, 83))
    for y in (-65, -25):
        carriage = carriage.fuse(box((18, 10, 1), (0, y, 85.5)))
    add("deployment_carriage", carriage, "carriage")

    # Rear and moving clevises have explicit M3 clearance holes and metal pins.
    def clevis(name, x, y, z, group, attach_z):
        cheeks = []
        for off in (-9, 9):
            c = drilled_block(
                (10, 3, abs(attach_z - z) + 6),
                (x, y + off, (attach_z + z) / 2),
                [(1.6, 6, (x, y + off - 3, z), (0, 1, 0))],
            )
            cheeks.append(c)
        cheeks.append(box((10, 21, 4), (x, y, attach_z)))
        clevis_shape = cheeks[0].fuse(*cheeks[1:]).clean()
        add(name, clevis_shape, group)
        add(
            name + "_pin",
            cylinder(1.5, 24, (x, y - 12, z), (0, 1, 0)),
            group,
            "metal",
            "M3 pin envelope; retention hardware pending",
        )

    clevis("deploy_rear_mount", -60.013, -45, 108, "fixed", 129)
    add("rear_mount_bridge", box((14, 49, 5), (-60.013, -67.5, 132)), kind="metal")
    clevis("deploy_rod_mount", 0, -45, 108, "carriage", 83)
    clevis("clamp_rear_mount", 0, -45, 67.013, "carriage", 80)
    # Vertical guides carry the finger moment, separate from the actuator rod.
    for i, x in enumerate((-10, 20)):
        add(
            f"clamp_guide_{i}",
            cylinder(1.5, 76, (x, -45, 4), (0, 0, 1)),
            "carriage",
            "metal",
            "3 mm shaft envelope; bearing supplier open",
        )
        add(
            f"clamp_guide_top_{i}",
            drilled_block((8, 10, 30), (x, -45, 66), [(1.55, 32, (x, -45, 50), (0, 0, 1))]),
            "carriage",
        )
        b = cylinder(3, 10, (x, -45, 12), (0, 0, 1)).cut(cylinder(1.6, 12, (x, -45, 11), (0, 0, 1)))
        add(f"clamp_bearing_{i}", b, "jaw", "purchased", "Bearing envelope")
    yoke = box((38, 14, 4), (5, -45, 13))
    for x in (-10, 20):
        yoke = yoke.cut(cylinder(3.05, 6, (x, -45, 10), (0, 0, 1)))
    add("lower_yoke", yoke, "jaw")
    clevis("clamp_rod_mount", 0, -45, 17, "jaw", 9)
    add("finger_carrier", box((14, 28, 5), (0, -26, 11.5)), "jaw")
    add(
        "finger_insert",
        box((10, 24, 1.5), (0, -8, -2.64)),
        "jaw",
        "metal",
        "Thin metal insert required; not a printed load-bearing edge",
    )
    add("finger_riser", box((6, 6, 17.5), (20, -18, 5)), "jaw")
    add("finger_side_bridge", box((27, 6, 5), (10, -18, 11.5)), "jaw")
    add("finger_under_bridge", box((30, 6, 1.5), (10, -18, -2.64)), "jaw", "metal")
    add(
        "contact_pad",
        box((10, 8, 0.75), (0, 0, -1.515)),
        "jaw",
        "soft",
        "Replaceable pad; hardness, friction and compression unmeasured",
    )
    # Deliberately visible engineering reservation; not a fictional installed sensor.
    add(
        "force_sensor_reservation",
        box((20, 12, 6), (0, -28, 17)),
        "jaw",
        "envelope",
        "Space claim only; force path/compliance cartridge not yet designed",
    )

    assembly = cq.Assembly(name="hybrid_tool_closed_fit_review")
    custom = cq.Assembly(name="custom_parts_closed_fit_review")
    envelope = cq.Assembly(name="assembly_with_actuator_envelopes")
    records = []
    (output / "parts").mkdir()
    (output / "print_fit_only").mkdir()
    for p in parts:
        name, shape = p["name"], p["shape"]
        color = cq.Color(*colors[p["kind"]])
        assembly.add(shape, name=name, color=color)
        public_shape = shape
        if "actuator" in name:
            b = shape.BoundingBox()
            public_shape = box(
                (b.xlen, b.ylen, b.zlen),
                ((b.xmin + b.xmax) / 2, (b.ymin + b.ymax) / 2, (b.zmin + b.zmax) / 2),
            )
        envelope.add(public_shape, name=name, color=color)
        cq.exporters.export(shape, str(output / "parts" / (name + ".step")))
        cq.exporters.export(
            shape, str(output / "parts" / (name + ".stl")), tolerance=0.06, angularTolerance=0.15
        )
        if p["kind"] == "printed":
            custom.add(shape, name=name, color=color)
            cq.exporters.export(shape, str(output / "print_fit_only" / (name + ".stl")), tolerance=0.03)
        b = shape.BoundingBox()
        records.append(
            {k: v for k, v in p.items() if k != "shape"}
            | {
                "color": colors[p["kind"]],
                "valid_brep": shape.isValid(),
                "bounds_mm": [[b.xmin, b.ymin, b.zmin], [b.xmax, b.ymax, b.zmax]],
                "solid_count": len(shape.Solids()),
                "volume_mm3": shape.Volume() if shape.Solids() else None,
            }
        )
    assembly.save(str(output / "assembly.step"))
    envelope.save(str(output / "assembly-envelope.step"))
    custom.save(str(output / "printed-parts.step"))
    report = {
        "status": "FIT_REVIEW_ONLY",
        "units": "mm",
        "manufacturer_step_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "source_url": "https://www.actuonix.com/assets/images/datasheets/PQ12_NEWSHAFT_STP.zip",
        "actuator_candidate": "2 x PQ12-30-6-P",
        "actuator_stroke_mm": 20,
        "deployment_used_mm": 18,
        "clamp_review_used_mm": 18,
        "clamp_extension_at_closed_mm": 8,
        "closed_cable_gap_mm": 0.14,
        "parts": records,
        "open_items": [
            "force sensing and series compliance load path",
            "hardware fasteners and retention",
            "exact guides and running fits",
            "FR3 adapter and inertia",
            "hose routing and ESD",
            "collision and stiffness qualification",
            "actuator low-speed/force qualification",
        ],
        "scope": (
            "Kinematic layout with authentic actuator CAD. "
            "No grasp physics, manufacturing release or collision-free claim."
        ),
    }
    (output / "assembly.json").write_text(json.dumps(report, indent=2) + "\n")
    print(
        json.dumps(
            {"parts": len(parts), "valid": all(r["valid_brep"] for r in records), "output": str(output)}
        )
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    build(a.source, a.output)
