"""Check result metadata against decoded video and recorded simulation state."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    hashes_path = args.run / "source/sha256.json"
    hashes = json.loads(hashes_path.read_text())
    for relative, expected in hashes.items():
        actual = hashlib.sha256((args.run / "source" / relative).read_bytes()).hexdigest()
        assert actual == expected, f"Frozen source changed: {relative}"
    result = json.loads((args.run / "results.json").read_text())
    trajectory = json.loads((args.run / "trajectory.json").read_text())
    assert trajectory, "Missing simulated trajectory"
    assert all(np.isfinite(sample["joint_positions"]).all() for sample in trajectory)
    assert all(np.isfinite(sample["cable_xyz_m"]).all() for sample in trajectory)
    audit = result.get("timestep_audit")
    if audit:
        assert audit["requested_steps"] == audit["observed_physics_callbacks"], "Physics step count mismatch"
        assert np.isclose(
            audit["observed_simulated_time_s"], audit["expected_simulated_time_s"], atol=1e-6
        ), "Physics clock audit failed"
    probe = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-count_frames",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=width,height,nb_read_frames,r_frame_rate",
            "-of",
            "json",
            str(args.run / "experiment.mp4"),
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    stream = json.loads(probe.stdout)["streams"][0]
    assert (stream["width"], stream["height"]) == (1280, 480)
    frames = int(stream["nb_read_frames"])
    assert frames > 0
    if result["status"] == "PASS":
        assert frames == result["video_frames"], "Video metadata disagrees with decoded frames"
    phases = json.loads((args.run / "experiment.phases.json").read_text())
    assert sum(p["frames"] for p in phases) == frames
    clips = sorted((args.run / "stage-clips").glob("*.mp4"))
    assert len(clips) == len(phases)
    for clip in [args.run / "experiment.mp4", *clips]:
        subprocess.run(
            ["ffmpeg", "-v", "error", "-xerror", "-i", str(clip), "-f", "null", "-"],
            check=True,
            capture_output=True,
        )
    report = {
        "artifact_checks": "PASS",
        "verified_source_hashes": len(hashes),
        "experiment_status": result["status"],
        "decoded_video_frames": frames,
        "stage_clips": len(clips),
        "trajectory_samples": len(trajectory),
        "physics_clock_audited": bool(audit),
    }
    (args.run / "artifact-checks.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report))


if __name__ == "__main__":
    main()
