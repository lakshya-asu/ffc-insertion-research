# Arducam B0498 simulation reference

The user selected the B0498 reference on 2026-09-27. This fixes the simulation specification; it does not identify the user's physical camera. The selected package is Sony IMX585 with a manual 16 mm C-mount lens and a 3840 × 2160, 15 fps USB3 YUY2 mode.

Source: [manufacturer listing](https://www.arducam.com/presalesarducam-8-3mp-imx585-manual-focus-usb-3-0-camera-module-with-16mm-c-mount-lens.html). The listing describes rolling shutter and default focus at 5 m to infinity, adjustable manually. It does not establish usable focus at our candidate mounting distances.

## Optical assumptions

`config/arducam-b0498.json` declares a centered native-pitch crop: 3840 × 2160 pixels at 2.9 µm, or 11.136 × 6.264 mm. A rectilinear 16 mm projection gives fx = fy = 5517.24 pixels and a 38.4° horizontal angle. The manufacturer reports 41°; measured intrinsics must resolve this discrepancy. We have not calibrated crop/ISP behavior, distortion, blur, exposure, rolling readout or noise. DOF remains disabled, so the frames establish geometry, not near-focus sharpness.

OpenUSD uses tenths of scene units for focal length and aperture. `ffc.camera_optics.apply_reference_optics` converts millimetres using the stage's metres-per-unit, sets fixed focus distance, and preserves the physical crop aspect ratio. Tests independently inspect the USD frustum on metre and centimetre stages and verify downsampled preview scaling.

## Placement and evidence

`config/arducam-b0498-cell-v1.json` records four candidate positions: overhead pickup, end presentation, an entrance-facing view offset about 11° from board normal, and an east side view. All four use the same selected camera reference. They are candidate mounts, not a confirmed count of purchased cameras or collision-qualified placements.

At the entrance-facing position, target distance is about 319 mm and normal-plane horizontal field about 222 mm, giving 17.3 native pixels/mm. This replaces the much narrower assumed field from the earlier entrance audit. Pixels/mm is sampling, not detector accuracy; board-plane sampling also depends on viewing angle. The 28 mm historical study is preserved as an optical requirement comparison.

Run `outputs/arducam-b0498-001` contains 16 native 4K RGB frames, configuration, camera metrics and USD. Four authored cable situations expose visibility; jaw blocks are envelopes, not simulated grasps. Timeline elapsed is zero and motion remains disabled. The public `docs/live/arducam-study` contains pixel-identical lossless WebP copies, a capture record and a 16-second review montage. No annotation plane enters these RGB frames. Existing model scores are unchanged and do not evaluate these new optics.

The live renderer now uses C1 for `desk` and C3 for `board`, preserving their field at 960 × 540. `workcell` remains a separate human observer camera. The 15 fps specification is not a claim about live rendering or policy frequency.

## Reproduction

Stop the preview using the documented `LAB_LIVE.md` stop flag before rendering. Use a new output directory:

```bash
docker compose run --rm experiment scripts/review_sensor_layout.py --config /workspace/config/arducam-b0498-cell-v1.json --output /workspace/outputs/arducam-b0498-NEW
.venv/bin/python scripts/publish_sensor_review.py outputs/arducam-b0498-NEW docs/live/arducam-study
```

The renderer's default remains the historical sensor study for reproducibility; pass the explicit Arducam config for new reference captures. The live renderer selects Arducam by default. Next: measure real focus reach and intrinsics, then qualify entrance visibility and native-resolution boundary estimates before retraining. No controller, actor, critic, reward or transition consumes simulator truth.
