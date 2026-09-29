# 034 — Cable-end contact exclusion

This is an offline planar geometry study supporting Step 2. It does not move Isaac, alter the existing assembly, or authorize runtime motion. Gripper choices remain open in [the options register](../design/compact-tool/options.md).

The review reads the mini-end width, contact length and stiffener length from `config/pi-zero-task.json`. The 4 mm exposed length, 6 mm stiffener and material thicknesses remain unmeasured assumptions. We exclude the full width of the exposed terminal strip on both faces for a pinch, so the upper stiffener cannot conceal a lower-pad/contact collision.

The current pads have a 6 × 3 mm footprint. The initial review uses ±0.5 mm independent placement error, ±10° in-plane yaw and a 1 mm exclusion margin. These are exploratory design bounds, not measured perception error or damage tolerances. Rectangle bounds cover the continuous yaw interval analytically. A 7 mm circle represents the candidate suction cup's compressed outer envelope; sealing and pressure are not modeled.

| Placement, center distance from leading edge | Result under base assumptions |
| --- | --- |
| Pad at 2.5 mm | Rejected: exposed-contact region |
| Pad at 5 mm | Rejected: overlaps contact region despite being centered within stiffener length |
| Pad at 10 mm | Clears contact exclusion and lies on insulated body |
| Cup at 12 mm | Clears exclusion and lies on body; only 0.75 mm lateral clearance remains after placement error and margin |

The pad and cup placements are separate alternatives, not a simultaneous assembly layout. Their footprints can overlap: this figure is not a verified handoff mechanism.

A 324-case design sweep varies contact length (3/4/5 mm), stiffener length (5/6/8 mm), placement bound (0.25/0.5/1 mm), yaw bound (0/5/10°) and pad setback (5/8/10/12 mm). For each setback there are 81 combinations. Exclusion clears in 0, 60, 81 and 81 combinations respectively; the footprint is also wholly on body in 0, 6, 54 and 81. These are deterministic sensitivity counts, not success rates or sampled manufacturing distributions.

Source, controller-independent geometry helper, full JSON, PNG and PDF are preserved in `outputs/grasp-footprint-review-001`. Seven unit cases check contact exclusion, uncertainty-induced rejection, the interior angular maximum, body/cup placement and invalid uncertainty. Targeted tests and lint pass.

```bash
.venv/bin/python scripts/review_grasp_footprints.py --output outputs/grasp-footprint-review-NEW
.venv/bin/pytest -q tests/test_grasp_footprint.py
```

Next: use these separate footprints to lay out the integrated handoff and the passive fixture, test swept clearances, and model the flexible cable. Longer setback increases the unsupported leading end; the planar result cannot establish buckling resistance or acceptable clamping pressure. Do not convert this offline map into simulator-state runtime targeting: runtime grasp placement must be estimated from images with uncertainty.
