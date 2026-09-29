# Held-end inspection — capture available

Docker access is restored. The prepared Isaac capture completed with seven raw frames. [Open the review](capture-001/index.html), video and native replay. Inspection is not qualified: focus varies, the tool reference is cropped and the terminal clips near the final pose.

## What is ready

- RGB-only appearance-region baseline with separate terminal/tool candidate measurements. It always leaves leading edge and slip unverified and motion disabled.
- Image ingress reuses calibration identity, shape, timestamp and sequence checks. No cable pose or simulator label is accepted as a camera input.
- Saved-view review with two existing human-view RGB images and four explicitly synthetic image edits. These are neither independent held-out cases nor calibrated macro-camera measurements.
- A fixed-camera capture recipe using the existing Basler/Kowa optical reference, with raw RGB, intrinsic matrix, optical-frame transform, explicit replay clock and checksums.
- Five CPU regressions covering ambiguity, absent regions, image translation, clipped regions, stale delivery and unexpected truth fields.

## What the current review found

The colour-only baseline confuses the blue adapter/tool surfaces with the terminal in the saved views and rejects them as ambiguous/clipped. The yellow-region reference can also change under occlusion and illumination. This baseline has not localized the leading edge and is not a substitute for the learned feature pipeline. Its image extents are diagnostic only.

## Repeat the capture in Isaac

From the project root, use a fresh output directory:

```bash
COMPOSE_IGNORE_ORPHANS=1 docker compose run --rm experiment \
  scripts/capture_held_end_inspection.py \
  --stage /workspace/docs/library/fr3-flex-008/replay.usdz \
  --config /workspace/config/held-end-inspection-v1.json \
  --output /workspace/outputs/held-end-capture-NEW
```

This renders recorded state; it does not rerun physics. Review `failure.txt` and `manifest.json`, not just container exit status. The output contains synthetic camera calibration, not physical calibration. The camera is proposed at 45° elevation, looking from the free-end side toward the known presented station; its mount and clearance are not qualified.

Rebuild this saved-image review with:

```bash
uv run python scripts/review_held_end_images.py --output outputs/held-end-review-new
```

## Review gates still open

1. Inspect every fresh camera frame at the known working distances. Reject frames where the terminal/leading edge is hidden, out of focus or clipped.
2. Independently label visible leading-edge endpoints, visibility/occlusion and terminal side. Keep labels outside the inference interface. Missing edges receive an explicit unknown label.
3. Compare learned features and classical image processing on complete held-out scenes. The existing entrance head is not assumed compatible with this camera or held-cable distribution.
4. Estimate relative end pose with uncertainty. A centroid shift may reflect perspective, deformation or occlusion; it is not proof of mechanical slip.
5. Test timing/calibration failures before connecting inspection to a motion transition. Keep the current pad/encoder guard active. Alignment and insertion remain disabled.

Current validation: all 178 CPU tests pass. Seven Isaac frames were inspected, their file/calibration identities checked, and the camera scene packaged as a physics-disabled USDZ. No new physics or robot control was executed. The earlier failed saved-image baseline remains in historical.html.

Rebuild the camera review in a fresh directory with `.venv/bin/python scripts/review_held_end_capture.py --capture outputs/held-end-capture-001 --output outputs/held-end-review-NEW --ffmpeg /path/to/ffmpeg`. Select `/World/Cameras/HeldEndInspection` in the downloaded USDZ.
