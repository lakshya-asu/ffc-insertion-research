# 039 — Mounted flexible-cable lift commissioning and replay library

29 September 2026. This is a failed/unfinished loaded-motion qualification, not insertion.

The compact CAD tool is attached to FR3 link 7 through the modelled adapter. Seven measured arm angles provide tool kinematics; ideal pad impulse magnitudes and jaw encoders drive the existing pinch skill. The reference is a bounded 10 mm vertical path from a known presented cable pose. Cameras inspect the run; ROS and camera control are not connected. Exact cable poses are recorded only for offline review.

## Findings

- `outputs/fr3-flex-001`: contact lost. Initialisation allowed the vendor reset posture to disturb the cable; wrist/desk placement also required correction.
- `outputs/fr3-flex-002`: no lift; trial timed out with lower/upper loads around 0.049/0.099 N. Contact audit exposed approximately 7 kN impulse-derived spikes between screw-clearance envelopes and opposite-jaw guide CAD. These nonphysical envelopes were mistakenly included as solid collision geometry.
- `outputs/fr3-flex-003`: excludes screw envelopes, retaining pad and physical component collisions. It established bilateral contact and began lifting, then entered `contact_lost` at roughly 0.14 mm measured lift. This does not qualify the requested 10 mm lift. Review retained traces before changing any thresholds.

The mounted script now saves initial and last scenes, including on an in-trial exception. Run directories preserve source snapshots, sensor traces, separate cable pose traces, contact peaks and videos. Internal guide collision filters, approximate inertias, ideal gravity compensation and convex component hulls remain modelling assumptions.

## Learning infrastructure delivered alongside commissioning

- `research/learning/PLAN.md` is the maintained seven-skill execution plan, including imitation before contact RL and independent evaluation.
- `ffc.learning_episode` exports only the declared pinch sensor/action fields. It rejects unknown sensor fields, invalid values and mismatched/nonmonotonic timestamps. Missing cameras and action application timestamps are explicit.
- `scripts/archive_pinch_replay.py` packages experiment 038 bench runs as physics-disabled USDZ timelines. It checks every captured cable position against the reopened package. These are recorded-state inspection assets, not executable physics experiments.
- `scripts/check_recorded_replay.py` opened the positive package in Isaac 6.1.0 and rendered its start/end timeline frames. The images show the recorded lift. This is a headless playback check, not an interactive GUI test.
- `/library/` links positive, empty and feedback-dropout videos, native replays, sensor episodes, reports and manifests. Full rerun bundles and native FR3 replay export remain pending.

## Reproduce the archive operation

```bash
uv run python scripts/archive_pinch_replay.py outputs/flex-pinch-001 outputs/new-replay-archive
COMPOSE_IGNORE_ORPHANS=1 docker compose run --rm experiment \
  scripts/check_recorded_replay.py \
  --stage /workspace/outputs/new-replay-archive/replay.usdz \
  --output /workspace/outputs/new-replay-check
```

Use fresh output paths. The source bench run and referenced CAD must be present for export; the resulting packaged USDZ is portable. Exact source hashes and artifact hashes are recorded in its manifest. Physics reproducibility across drivers/GPUs is not established.
