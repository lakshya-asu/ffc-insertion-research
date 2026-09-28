# Image-space alignment from predicted entrance features

The next component fits entrance-rim and cable-leading-edge lines using only the frozen model's class mask. It returns projected normal separation in pixels and relative angle in image-plane degrees, or abstains with a reason. It does not estimate depth, physical clearance, insertion pose or a safe command. This component is currently an offline diagnostic; the live ROS demo continues to publish segmentation.

## Data and code boundary

`src/ffc/image_alignment.py` has no USD, scene pose, model weight or annotation dependency. `scripts/estimate_image_alignment.py` reads prediction masks and input identities, then writes estimates before scoring. The separate `score_image_alignment.py` reads authored poses only to project independent reference edges. Its assumed socket/stiffener dimensions match the capture geometry; they are not manufacturer-certified dimensions.

The estimator selects the largest component of each rim and the leading band, checks pixel count, dominant-component fraction, span, coverage and image-boundary contact, then fits the inward-facing rim boundaries and the front of the leading band. It trims the outer 10% of support for the line fit and uses robust residual rejection. Rims must have consistent orientation and ordering with a plausible apparent gap. Measurements require sufficient common horizontal support.

The mounted camera's orientation determines which image boundary is the front edge. This is a fixed-view protocol, not a viewpoint-invariant detector. It does not verify endpoints or infer lateral centering from component centroids. Closed-slider scenes can still yield image measurements; the estimator does not authorize approach or classify latch readiness.

`config/image-alignment-v1.json` was written before the batch evaluation, and thresholds were not changed in response to its results. It defines diagnostic heuristics, not confidence probabilities or insertion tolerances. `calibrated_uncertainty` remains null. The 60 scenes had already been inspected during earlier work, including a projection sanity check during this milestone; this is **retrospective development evidence**, not a fresh independent qualification set.

## Outcome

35 of 60 scenes returned image measurements. All 24 missing-target or short-grasp hidden-tip cases abstained. One additional near-entry image, `0021-macro.png`, abstained because the predicted lower rim was fragmented. No model was retrained.

For each accepted line, the offline scorer compares perpendicular displacement over the overlap between predicted support and the independently projected physical edge. It does not certify endpoint accuracy. The table reports the median per-image mean displacement and the largest sampled displacement in any accepted image.

| Edge | Accepted views | Median mean error, px | Worst sampled error, px | Worst image-angle error |
|---|---:|---:|---:|---:|
| Upper entrance rim | 35 | 0.571 | 4.602 | 0.592° |
| Lower entrance rim | 35 | 0.532 | 4.663 | 0.508° |
| Cable leading edge | 35 | 0.391 | 1.491 | 0.110° |

The worst upper/lower rim errors occur in bright views `0025-macro.png` and `0035-macro.png`. A low line-fit residual can coexist with biased localization. These results do not support treating fit residuals as uncertainty bounds. A separate calibration set and a fresh test set are required before a quantitative uncertainty claim.

The reference geometry is the authored substitute connector and rigid cable. Errors against it test synthetic localization only. The result does not establish real optical accuracy, mechanical tolerances or robustness to realistic cable deformation.

## Verification and review

Seven new tests cover known offset and tilt, absent features, fragmentation, curvature, image clipping and crossed rims. The complete CPU suite passes 113 tests. All 60 overlays were visually inspected in contact sheets. The [demo geometry section](https://lakshya-asu.github.io/ffc-insertion-research/demo/#alignment) exposes every case, measurements, reasons for abstention and a video of the review.

Two presentation failures were caught before publication: a duplicate manifest key stopped the first publisher run, and using an already resized web image misplaced the display overlays. Failed artifacts remain under `outputs/image-alignment-001/`. The corrected publisher transforms native RGB to the exact 1232 × 1024 coordinate grid before drawing lines. These presentation fixes did not change estimator output or evaluation metrics.

## Reproduce

```bash
PYTHONPATH=src .venv/bin/python scripts/estimate_image_alignment.py \
  outputs/macro-dinov3-predictions-001 config/image-alignment-v1.json \
  outputs/image-alignment-NEW/estimates.json
PYTHONPATH=src .venv/bin/python scripts/score_image_alignment.py \
  outputs/macro-dinov3-test-001 outputs/image-alignment-NEW/estimates.json \
  outputs/image-alignment-NEW/evaluation.json
```

Both entrypoints require fresh output files. Run 001 preserves estimator/protocol SHA-256 identities, prediction identities and the first evaluation; the second report additionally includes projected references for visualization.

## Next gate

Keep this image-only diagnostic separate from motion control. The next experiment should test repeatability, wider occlusion and viewpoint changes on newly captured scenes, and quantify uncertainty on a separate calibration split. Metric alignment also needs an observable depth/pose model, verified correspondence to physical entrance boundaries, and appropriately synchronized camera/robot measurements. Contact mechanics remain a separate prerequisite for insertion.
