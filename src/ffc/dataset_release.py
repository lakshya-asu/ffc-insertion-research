"""Fail-closed evidence gate for a future simulation skill dataset.

This audits a review manifest. It does not certify the science in an attached
report, capture episodes or enable the historical training scripts.
"""

import hashlib
import json
from pathlib import Path

REQUIRED = {
    "pickup": {"bending_convergence", "contact", "pickup", "sensor_boundary"},
    "insertion": {"bending_convergence", "contact", "pickup", "sensor_boundary", "insertion", "recovery"},
}


def review_release(spec: dict, manifest: dict, evidence_root: Path) -> dict:
    """Reports must be hash-pinned, reviewed and tied to this exact profile."""
    skill = manifest.get("skill")
    if skill not in REQUIRED:
        raise ValueError("Unsupported skill release")
    reasons = []
    if manifest.get("schema_version") != 1:
        reasons.append("Unsupported manifest schema")
    if manifest.get("profile_sha256") != spec["sha256"]:
        reasons.append("Manifest cable profile mismatch")
    root = evidence_root.resolve()
    for gate in sorted(REQUIRED[skill]):
        entry = manifest.get("evidence", {}).get(gate)
        if not isinstance(entry, dict):
            reasons.append(f"{gate}: missing evidence")
            continue
        try:
            path = (root / entry["path"]).resolve()
            if not path.is_relative_to(root):
                raise ValueError("Evidence must be inside its archive")
            raw = path.read_bytes()
            if hashlib.sha256(raw).hexdigest() != entry["sha256"]:
                raise ValueError("Evidence hash mismatch")
            report = json.loads(raw)
            if report.get("profile_sha256") != spec["sha256"]:
                raise ValueError("Evidence cable profile mismatch")
            if report.get("gate") != gate or report.get("passed") is not True:
                raise ValueError("Gate did not pass")
            if not isinstance(report.get("review"), str) or not report["review"].strip():
                raise ValueError("Missing acceptance review")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            reasons.append(f"{gate}: {exc}")
    return {
        "skill": skill,
        "profile_sha256": spec["sha256"],
        "evidence_gate_passed": not reasons,
        "reasons": reasons,
        "scope": "Simulation evidence only; episode integrity and policy evaluation are separate gates",
    }
