# Mounted macro dataset and the grasp-setback visibility tradeoff

The current Basler/Kowa camera at 45° now has its own native RGB capture pipeline with the complete offset tool and offline FR3 IK placement. This continues Step 1 in simulation. No physical grasp, motion rollout, trained macro prediction or insertion success is claimed.

## Audit before training

`outputs/macro-dataset-audit-001` contains 30 independent static scenes, three per condition: ordinary, near entry, board rotation, short grasp, dim, bright, focus drift, absent cable, absent socket and empty targets. The capture seed is 930101. All 30 RGB frames were visually inspected in three contact sheets. The short-grasp cases exposed complete loss of leading-band visibility.

The full native image is 2448 × 2048. Labels distinguish background, upper entrance rim, lower entrance rim, the 0.3 mm leading band, open slider and closed slider. Label surfaces retain their normal materials; label colors never enter RGB. The source geometry and optical limitations from experiments 017–019 remain unchanged.

The independent validator checks dimensions, class range, RGB/mask hashes, no duplicate RGB, recorded class counts, slider consistency and strict empty labels for absent targets. It independently projects a conservative leading-band envelope into the calibrated optical frame and rejects labels outside it. A deliberate tamper probe added a false band pixel and updated both its hash and class counts; projection containment still rejected it (`macro-label-tamper-001`). This catches an annotation failure that hash validation alone would miss. Containment does not establish pixel-perfect completeness.

## Matched setback experiment

`outputs/macro-setback-pairs-001` contains three matched pairs. Each pair has identical board transform, cable transform, latch state, camera, focus and illumination. Only the tool reference setback changes from 6 mm to 10 mm; offline IK reposes the complete tool and robot. Exact equality of the recorded scene variables was checked.

| Pair | Leading-band pixels, 6 mm | Leading-band pixels, 10 mm |
|---|---:|---:|
| 1 | 0 | 49,387 |
| 2 | 0 | 51,443 |
| 3 | 0 | 51,599 |

The tool blocks the band at 6 mm in these three scenes. A nominal 10 mm setback remains the development reference, with 6 mm examples retained as explicit occlusion challenges. This small geometric comparison does not establish optical resolution, a probability of visibility, a mechanically stable grasp or an optimal setback. Cable bending and grip-slip tests must evaluate the unsupported-length cost separately.

## Development protocol

`outputs/macro-dataset-dev-001` uses seed 930201 for 160 scenes: 128 training and 32 validation. Conditions repeat in a balanced cycle with independently drawn small pose variations and latch states. No audit or matched-pair image is a training/validation sample. The native sensor directory contains only RGB and fixed camera calibration. Authored object poses, masks and class counts live under `offline/` and are excluded from runtime inference.

Board translation varies by ±0.25 mm X, ±0.4 mm Y and 0–0.15 mm Z. Board yaw varies by ±0.6° ordinarily and ±3° in the rotation condition. Cable yaw varies by ±2°, lateral displacement by ±0.4 mm, and height offset by 0–0.2 mm. Near-entry gaps are 0.5–1 mm; other conditions use 1–6 mm. These are declared sensitivity ranges, not identified manufacturing tolerances or a complete deployment distribution.

A conservative leading-corner check retains at least 0.1 mm before the opening plane. Each static tool pose must maintain a 5 mm separating-axis bound against the camera/mount boxes. This does not check every arm/PCB/desk/self-collision or qualify a trajectory. The complete tool remains a design envelope, the cable is rigid in this visibility experiment, and the board supports are not a deformable fixture model.

Dim and bright cases scale only the dome-light intensity to 0.12× and 4×; other lights remain. Focus drift changes the renderer focus distance by ±0.8 mm. No calibrated exposure, MTF, Bayer noise or photometric model is implied. These are not a physical glare qualification. The dataset has no varied cable curvature, contact deformation, material appearance or broad scene diversity yet.

Training will use the exact ROS frontend revision `macro-rectify-area-half-pad4-v1`, including full-view area resizing and transformed intrinsics. No new model has been selected. The independent final test must be captured only after weights, preprocessing and decision rules are frozen.

## Review and reproduction

The live page at `/live/#macro-data` provides every audit and matched-pair RGB/offline overlay, plus a 36-second montage of independent static scenes. It contains no learned prediction. Source CAD attribution follows the existing Zero and mount studies.

```bash
# Stop the separate live preview first using LAB_LIVE.md.
docker compose run --rm experiment scripts/capture_mounted_macro.py \
  --split audit --count 30 --output /workspace/outputs/NEW-audit
.venv/bin/python scripts/validate_mounted_macro.py outputs/NEW-audit

docker compose run --rm experiment scripts/capture_mounted_macro.py \
  --split audit --count 6 --paired-setback --output /workspace/outputs/NEW-pairs

docker compose run --rm experiment scripts/capture_mounted_macro.py \
  --split development --output /workspace/outputs/NEW-development
.venv/bin/python scripts/validate_mounted_macro.py outputs/NEW-development
```

Fresh output directories are required. The dataset stores source snapshots offline, image/mask hashes and the source USD hash. The original audit source was recovered exactly and verified against its recorded SHA-256; the paired capture source is also archived. Subsequent captures snapshot their sources at startup. Zero elapsed physics is checked in every frame.

## Completed development checks

All 160 development frames passed annotation integrity checks. The split contains 128 training and 32 validation frames; no RGB hashes overlap the earlier audit/pair captures. The stored capture source matches its recorded hash. All 16 short-grasp development views also have an empty visible leading-band mask.

The exact ROS frontend accepted all 160 native RGB images and produced 1232 × 1024 observations in `outputs/macro-frontend-audit-002`. This check ran without networking, with only the sensor directory, the check script and a separate output directory mounted. An earlier frontend pass wrote into the dataset directory and is retained as a functional check, not an isolation test. Neither pass used labels as an input. No learned model was trained in this milestone.

The 97-test repository suite passes. Desktop and mobile browser checks loaded all 36 review scenes in both RGB and offline-overlay modes with no script errors or horizontal overflow. The 1600 × 760 montage decodes to 864 frames / 36 seconds. The native ROS live preview was restored after capture.
