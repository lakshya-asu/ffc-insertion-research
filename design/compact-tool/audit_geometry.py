"""Offline supplier geometry audit, in millimetres. Not a controller or contact test.

Run with the CadQuery environment and the original O-S supplier STEP. Component
indices are tied to its SHA256; a different revision requires another review.
"""

import argparse
import hashlib
import json
from pathlib import Path

import cadquery as cq

SOURCE_SHA256 = "c067fb10adedf664a4defb3c2f686acd64c55d2fb73ad32e9d11e52b9c74c90e"


def audit(source):
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if digest != SOURCE_SHA256:
        raise ValueError("Unreviewed supplier STEP revision")
    solids = cq.importers.importStep(str(source)).solids().vals()
    if len(solids) != 21:
        raise ValueError("Unexpected component count")
    lower, upper = solids[11], solids[16]
    # Inner planes at the front of each jaw, excluding chamfered front edges.
    faces = []
    for jaw in (lower, upper):
        candidates = [
            f
            for f in jaw.Faces()
            if f.geomType() == "PLANE"
            and abs(f.normalAt().z) > 0.999
            and -252 < f.Center().y < -248
            and 40 < f.Area() < 43
        ]
        if len(candidates) != 1:
            raise ValueError("Inner-face identification is ambiguous")
        faces.append(candidates[0].Center().z)
    imported_gap = faces[1] - faces[0]
    # Moving screw groups belong to their corresponding jaw. Fixed housing
    # remains unchanged. Test whole groups, not their overlapping AABBs.
    lower_group = cq.Compound.makeCompound(solids[11:16])
    upper_group = cq.Compound.makeCompound(solids[16:21])
    fixed = cq.Compound.makeCompound(solids[:11])
    rows = []
    gaps = sorted({1 + index * 0.5 for index in range(21)} | {1.14, 1.3})
    for gap in gaps:
        displacement = (imported_gap - gap) / 2
        lo = lower_group.translate((0, 0, displacement))
        hi = upper_group.translate((0, 0, -displacement))
        rows.append(
            {
                "nominal_base_gap_mm": gap,
                "each_jaw_offset_from_import_mm": displacement,
                "jaw_pair_overlap_mm3": abs(lo.intersect(hi).Volume()),
                "lower_fixed_overlap_mm3": abs(lo.intersect(fixed).Volume()),
                "upper_fixed_overlap_mm3": abs(hi.intersect(fixed).Volume()),
                "proposed_pad_gap_mm": gap - 1.0,
            }
        )
        print(f"gap {gap:.2f} mm checked", flush=True)
    return {
        "scope": "Offline CAD geometry only; no actuator, cable, contact or grasp physics",
        "source_sha256": digest,
        "component_count": len(solids),
        "coordinate_units": "mm in supplier frame",
        "jaw_axis": "Z",
        "front_attachment_plane_y_mm": -248.2103195204206,
        "inner_planes_z_mm": faces,
        "imported_gap_mm": imported_gap,
        "drawing_gap_range_mm": [1, 11],
        "rebase": "Translate lower jaw +Z and upper jaw -Z by (imported_gap - nominal_gap)/2; no scaling",
        "moving_components_zero_based": {"lower": list(range(11, 16)), "upper": list(range(16, 21))},
        "proposed_fingertips": {
            "status": "dimensional concept only; geometry and mounting not released",
            "inward_offset_each_mm": 0.5,
            "pad_gap_range_mm": [0, 10],
            "assumed_header_thickness_mm": 0.3,
            "base_gap_at_header_contact_mm": 1.3,
            "assumed_body_thickness_mm": 0.14,
            "base_gap_at_body_contact_mm": 1.14,
            "warning": "These are geometric contact positions, not safe force commands or a damage threshold",
        },
        "sampled_sweep": rows,
        "limitations": [
            "Sampled poses do not prove swept-volume clearance between samples",
            "Supplier CAD can contain deliberate overlaps and omitted internal mechanics",
            "No new fingers, nozzle, adapter, cable or board is included in the collision audit",
            "Ordering suffix conflicts between English catalogue and downloaded variants remain unresolved",
        ],
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("Preserve existing experiment results; choose a fresh output")
    result = audit(args.source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
