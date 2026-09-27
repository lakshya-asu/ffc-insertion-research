# Zero 2 W: camera-only visible-region prerequisite

The side-entry board now has a learned local component estimator. It is deliberately not an aperture estimator: the community CAD's latch state, slot clearance and physical mouth have not passed a real-part audit. The outputs are visible connector assembly, cable, mini-end stiffener/contacts and jaw envelope. Insertion pose, leading edge and latch state remain unknown; motion is disabled.

## Observation and model

Two B0498 / IMX585 / 16 mm reference cameras render 3840 × 2160 RGB. Each uses the same fixed optical-centre crop `[1472,856,2368,1304]`, preserving 896 × 448 native pixels. Crop choice was based on raw image inspection, not a perfect object box. This is a fixtured local working volume, not desk-wide discovery. The virtual macro is excluded.

A frozen official DINOv2 ViT-B/14 encoder supplies blocks 6 and 12 (1,536 concatenated channels). A lightweight decoder fuses projected features with a quarter-resolution RGB path and native-resolution RGB refinement. The ablation removes only the DINO feature branch and trains on the same images, optimizer schedule and losses. This measures the available feature representation's contribution within this experiment; it is not a controlled comparison against the historical Pi 4 U-Net, whose data and labels differ.

Official source and weight hashes are pinned in `config/dinov2-backbone.json`. The architecture is in `src/ffc/zero_region_model.py`; the contract is `config/zero-perception-v1.json`. Upstream source: https://github.com/facebookresearch/dinov2 (general model and code Apache 2.0).

## Data and split

Development `outputs/zero-perception-dev-004`: 80 independent static scenes, 160 images; 64 scenes/128 images train, 16 scenes/32 images validation. The cameras for a given scene stay in the same split. Both heads train for 40 epochs; maximum validation foreground mean IoU selects each checkpoint. Frozen checkpoints are in `outputs/zero-region-model-001`.

Test `outputs/zero-perception-test-001`: 36 fresh scenes, 72 images, captured after checkpoint selection. 20 scenes follow the development distribution (including periodic absent-object controls); 16 challenge scenes give two independent scenes per condition, viewed by both cameras. Conditions: absent board, absent cable, empty, dim, bright, large offset, large yaw and tool occlusion. Pooled condition rows include any controls from the test split as well as the challenges. This is a small diagnostic set, not a deployment failure-rate estimate.

Board position/yaw, cable gap/offset/yaw/roll, lighting and jaw envelopes vary. Supports follow the board and jaw roll follows the cable. All four scene lights are intensity-scaled for the lighting challenges; there is no calibrated exposure model. Cable/jaws are statically authored probes. There is no simulated grasp/contact or timeline advancement.

Failed development runs are preserved and excluded: 001 detected stale semantic labels in an absent-cable view; 002 was stopped after support/jaw transformations were found inconsistent; 003 was an interrupted restart before those fixes were applied. Run 004 uses unique scene paths, extra render settling and explicit absent-label checks. `validate_zero_dataset.py` checks image hashes, dimensions, mask class counts, absent controls, scene splits, class coverage and test/development hash separation. Hash separation does not by itself establish real-world diversity.

## Runtime boundary

`scripts/run_zero_inference.sh` launches a read-only Docker container with network disabled. It mounts only sensor RGB/metadata, frozen checkpoints, upstream backbone files, narrowly selected inference code and the output directory. It cannot read the scene, CAD, offline masks or offline pose labels. Calibration hash, image dimensions and recorded timestamp sequence are validated. Replay uses recorded time; this is not a demonstration of live wall-clock freshness.

All masks come from RGB. Connected components of at least 16 pixels become unverified review boxes, never mating or grasp targets. The separate offline scorer reads labels only after inference. Every output denies motion permission and leaves aperture/latch/leading-edge/insertion pose unknown. No actor, critic, reward, transition policy or tactile fusion is trained here.

## Evaluation interpretation

IoU is pooled per class; foreground mean excludes classes with neither predicted nor labelled pixels. Visible views require at least 16 labelled pixels. A false alarm requires a predicted connected component of at least 16 pixels on an absent-class view. Visible IoU ≥ 0.5 counts are reported separately, so correct empty views cannot inflate localization success. The optional silhouette boundary statistic measures the full component silhouette, not connector entrance accuracy; it is conditional on a nonempty prediction and must be read with miss counts.

Timing covers model compute on a decoded crop, excluding image capture/decode and component extraction. First-frame warmup is excluded. It does not establish a two-camera 15 fps robot loop. Public review windows are lossless WebP verified pixel-for-pixel against their RGB/overlay arrays. All held-out views and layers are available; the video selects the first four test images plus all 32 challenge images.

## Reproduction

Prerequisites: the Zero board/cell assets from experiment 013, the `ffc-isaac:6.1.0` and `ffc-perception:2.8.0-cu128` images, and the pinned backbone fetched by `scripts/fetch_dinov2_reference.py`. All output paths must be fresh. Stop the separate preview before capture.

```bash
set -e
.venv/bin/python scripts/fetch_dinov2_reference.py
docker compose run --rm experiment scripts/capture_zero_perception.py \
  --split development --output /workspace/outputs/NEW-development
.venv/bin/python scripts/validate_zero_dataset.py outputs/NEW-development
mkdir outputs/NEW-model
docker run --rm --gpus all --network none --user "$(id -u):$(id -g)" \
  -e PYTHONPATH=/code -e XFORMERS_DISABLED=1 \
  -v "$PWD/outputs/NEW-development:/data:ro" -v "$PWD/outputs/NEW-model:/results:rw" \
  -v "$PWD/third_party/dinov2:/backbone:ro" \
  -v "$PWD/scripts/train_zero_regions.py:/code/train_zero_regions.py:ro" \
  -v "$PWD/src/ffc/zero_region_model.py:/code/zero_region_model.py:ro" \
  ffc-perception:2.8.0-cu128 /code/train_zero_regions.py --data /data --output /results
docker compose run --rm experiment scripts/capture_zero_perception.py \
  --split test --output /workspace/outputs/NEW-test
.venv/bin/python scripts/validate_zero_dataset.py outputs/NEW-test --development outputs/NEW-development
scripts/run_zero_inference.sh outputs/NEW-test/sensor outputs/NEW-model outputs/NEW-predictions
.venv/bin/python scripts/evaluate_zero_regions.py outputs/NEW-test outputs/NEW-predictions outputs/NEW-evaluation.json
.venv/bin/python scripts/publish_zero_perception.py outputs/NEW-test outputs/NEW-predictions outputs/NEW-evaluation.json docs/live/NEW-review
```

The existing test is now disclosed. Further model selection needs new held-out scenes and, ultimately, a separate real-image test set.

## Next gate

Measure a real Zero socket and cable; confirm open latch geometry, working-distance focus, optical calibration and entrance visibility. Replace box jaws with a selected gripper/finger/tactile package and measure approach occlusion. Define physically verified mouth/rim/leading-edge labels, including unobservable cases. Only then train the finer geometry head, estimate calibrated uncertainty and evaluate localization against measured insertion clearance. Force/tactile retention, cable mechanics and insertion control remain separate future qualifications.

Validation selection: DINOv2 0.927616 foreground mean IoU, RGB ablation 0.705949. These are selection scores, not test results. Checkpoint SHA-256: DINO `24764d6ff4e481d964ccdab6ad8598779afa16b83bf2f668a2d947888e451fe3`; RGB `56bf7fb8d9466ea3226e311b3c9b205727356909fe4f0e1223bfcd9053f1a2de`.

## Frozen held-out result

| Model | Test (40 images) | Challenge (32 images) | All (72 images) | Median compute |
|---|---:|---:|---:|---:|
| DINOv2 + RGB detail | 0.8974 | 0.9092 | 0.9013 | 12.33 ms |
| RGB-only ablation | 0.6915 | 0.4911 | 0.5919 | 1.27 ms |

The test/challenge difference does not establish that challenges are easier: class visibility and pixel area vary, the set is small, and empty classes can have undefined IoU. DINO's entrance-camera mean is 0.8292 versus offset-camera 0.9357. These pooled scores do not measure multi-view fusion or calibrated depth.

DINO per-class IoU: connector 0.9159, cable 0.9733, mini end 0.7587, jaw 0.9572. At least 0.5 IoU was achieved in 61/62 visible connector views, 57/60 cable views, **36/59 mini-end views**, and 44/44 jaw views. Connector false alarms: 1/10 absent views. The other three classes had no component-level false alarm in this small set. The mini-end failures rule out treating this model as a reliable physical end localizer.

A second fresh isolated replay produced **144 byte-identical mask PNGs**, covering both models on all 72 frames. The scorer and all current repository tests pass (77 tests). Native RGB, both overlays, offline annotation and the full report are published at `/live/#zero-perception`. The inference worker has no simulator or annotation mount. This result advances component recognition only; no real-world or insertion claim follows.
