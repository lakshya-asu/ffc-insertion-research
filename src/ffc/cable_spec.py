"""Source-aware cable parameters for offline scene/mechanics generation."""

import hashlib
import json
import math
from pathlib import Path


def load_spec(path: str | Path) -> dict:
    raw = Path(path).read_bytes()
    spec = json.loads(raw)
    if spec.get("schema_version") != 1:
        raise ValueError("Unsupported cable specification")
    source_ids = {source["id"] for source in spec["sources"]}
    for name, parameter in spec["parameters"].items():
        value = parameter["value"]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError(f"Invalid value: {name}")
        if value <= 0:
            raise ValueError(f"Nonpositive parameter: {name}")
        status = parameter["status"]
        if status == "manufacturer_published":
            if not parameter["sources"] or not set(parameter["sources"]).issubset(source_ids):
                raise ValueError(f"Published parameter lacks a registered source: {name}")
        elif status == "design_assumption":
            lo, hi = parameter["exploratory_bounds"]
            if not (math.isfinite(lo) and math.isfinite(hi) and 0 < lo <= value <= hi):
                raise ValueError(f"Invalid exploratory range: {name}")
        else:
            raise ValueError(f"Unknown parameter provenance: {name}")
    v = values(spec)
    if not 0 < v["exposed_length_m"] <= v["stiffener_length_m"] <= v["wide_end_length_m"]:
        raise ValueError("Inconsistent terminal lengths")
    if not v["wide_end_length_m"] + v["transition_length_m"] < v["length_m"]:
        raise ValueError("Width transition exceeds cable")
    if not 0 < v["poisson_ratio"] < 0.5 or v["dynamic_friction"] > v["static_friction"]:
        raise ValueError("Invalid material assumptions")
    for end in ["mini", "standard"]:
        n = v[end + "_contacts"]
        if n != int(n) or n < 2 or (n - 1) * v[end + "_pitch_m"] >= v[end + "_width_m"]:
            raise ValueError("Invalid terminal contact geometry")
    return {**spec, "sha256": hashlib.sha256(raw).hexdigest()}


def values(spec):
    return {name: entry["value"] for name, entry in spec["parameters"].items()}


def width_at(spec, distance_from_mini_m):
    """Piecewise outline; transition location/length are declared assumptions."""
    v = values(spec)
    y = distance_from_mini_m
    if not math.isfinite(y) or not 0 <= y <= v["length_m"]:
        raise ValueError("Position outside cable")
    wide_start = v["length_m"] - v["wide_end_length_m"]
    t = max(0, min(1, (y - wide_start + v["transition_length_m"]) / v["transition_length_m"]))
    return v["mini_width_m"] + t * (v["standard_width_m"] - v["mini_width_m"])


def provenance_record(spec):
    return {
        "profile_id": spec["id"],
        "profile_sha256": spec["sha256"],
        "profile_revision": spec["revision"],
        "qualification": spec["qualification"],
        "assumed_parameters": [
            k for k, v in spec["parameters"].items() if v["status"] == "design_assumption"
        ],
    }
