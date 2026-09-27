# E010 — Refined Pi scene, learned RGB segmentation and live research view

The selected task is a Pi 4 Model B plus Camera Module 3 Standard and the Standard-Standard 200 mm ribbon. This experiment improves the static scene and starts supervised RGB perception. It does not move the robot, learn an action policy, or qualify real-world insertion.

[Live lab](https://lakshya-asu.github.io/ffc-insertion-research/live/) · [Hardware review](https://lakshya-asu.github.io/ffc-insertion-research/hardware/) · [Perception results](https://lakshya-asu.github.io/ffc-insertion-research/pi-perception/)

## Scene refinement

The Pi 4 community STEP remains pinned to the source recorded in E009. The converter's `--split-connectors` option separates each CSI/DSI connector's existing 17 solids into 15 metal contacts and two polymer parts. It preserves the original triangle geometry. Polymer colour assignments are visual approximations; the parts are not claimed to have calibrated latch roles or travel.

`extract_pi4_layout.py` reads the vector geometry in the official Pi 4 mechanical drawing and recovers small, closed, axis-aligned rectangular outlines. After rejecting overlap with existing major CAD parts, 102 footprint proxies remain. Their locations and XY envelopes come from that drawing; package types, heights and finishes are assumed. Seven vector labels add readable board context with approximate typography and placement. This is not an extracted BOM or manufacturer silkscreen artwork.

The refined import has 248 CAD leaves (51 Pi 4 + 197 Camera Module 3) and the same 779,957 CAD triangles. Procedural package and lettering geometry is additional. Seven actual RTX renders and a 48-frame, 8-fps orbit are published. The saved scene is `outputs/pi4-refined-002/raspberry-pi-workcell.usda` locally.

The cable preserves 200 mm centreline arc length, 16 mm width, 15 contacts at 1 mm pitch and opposite exposed faces. Appearance variation changes its shallow vertical/lateral bow and face-up orientation without flipping the entire arch below the desk. These remain authored static shapes, not a gravity or FEA solve. Pi support posts transform with the board during randomization.

## Observation and annotation boundary

Two fixed ideal pinhole views produce RGB8 at 768 × 512. The desk camera sees the cable and surrounding hardware; the board camera resolves the sockets and normally excludes the cable. Calibration records include proposed intrinsics and extrinsics, not physical camera calibration. Timestamps are host render completion on a monotonic clock, not synchronized hardware exposure.

Six segmentation classes are background, cable body, exposed contacts, stiffener, CSI and DSI. Ground-truth masks and approximate projected endpoint/socket-centre annotations stay in the offline dataset. The current model uses segmentation supervision only; the projected keypoints are not qualified insertion targets and are not supplied to inference.

The inference container is read-only, runs without network or GPU devices, drops capabilities, and mounts only a sensor directory, model directory, result directory and three Python files. `CameraGate` validates frame metadata and immutable RGB buffers. The network receives only RGB scaled to [0,1]. Camera identity and calibration data are not neural features. No USD, label directory, simulator socket or controller is mounted.

Inference emits semantic masks and descriptive connected-component centroids with uncalibrated softmax scores. It always records `motion_permitted=false`. There is no action policy or deployable uncertainty gate in this milestone. Simulator labels are read by a separate offline scorer after inference.

## Development and independent evaluation

The corrected development run is `outputs/pi-perception-dev-004`: 96 training scenes and 24 validation scenes, each with both camera views (192 + 48 images). Both views of a scene stay in the same split. The seed is 927101.

A small U-Net has encoder widths 12/24/48/96 and three skip-connected decoder levels. Training uses weighted cross-entropy plus foreground Dice loss, AdamW, a cosine schedule, batch size four and 40 epochs. Augmentation applies flips, brightness gain and small RGB noise. These augmentations are not a measured sensor model. No pretrained foundation model or action model is used.

The seed is 927311. CUDA training requested deterministic algorithms with warnings, but PyTorch reports a nondeterministic cross-entropy kernel. The seed and software versions aid reproduction; bitwise-identical GPU training is not claimed. The published checkpoint is selected only by maximum validation foreground mean IoU.

The corrected model run is `outputs/pi-perception-model-002`. A hash and freeze timestamp are written before the test capture. The test seed is 927203: 40 ordinary scenes plus 16 challenge scenes, two views each (112 images). Challenges are missing cable, occlusion, blue distractor, reduced dome illumination, removed board markings, missing board, increased dome illumination and larger bow. There are only two scenes per challenge; this is not enough to estimate deployment reliability. Other lights remain active in the dome-intensity challenges.

All splits share the CAD assets, ideal cameras and renderer. The test therefore measures synthetic pose/appearance variation, not transfer to unseen physical boards or real cameras. Future model changes need explicit development data and fresh evaluation protocols; do not silently tune on this benchmark.

## A failed run caught by the audit

The initial model run, `pi-perception-model-001`, was stopped at epoch 17. RGB showed the ribbon but all cable/contact/stiffener annotations were empty. It is invalid and was not tested. Diagnostic captures showed stale semantic mappings when geometry, visibility and exposed faces changed under reused USD prim identities. A visible-first case alone did not fix subsequent face changes.

Giving each cable pose a fresh prim identity restored all three classes across the diagnostic sequence and the full recapture. The capture now rejects ordinary desk views with missing cable classes. `validate_pi_dataset.py` checks every RGB hash, shape, mask count, ordinary cable class presence, absent-object consistency and per-split class coverage. The training script separately rejects any entirely empty foreground class before optimization. A regression test reproduces the missing-label failure.

The corrected development audit records 566,041 cable pixels, 9,732 contact pixels and 18,931 stiffener pixels in training; validation has 137,970, 2,404 and 4,599 respectively. These are annotation coverage checks, not claims of physical accuracy. The public work log retains the failure and correction.

The first test capture, `pi-perception-test-001`, also failed the audit: making child details visible had restored the parent board in an intended missing-board case. No scores from that capture were reviewed or used to change the model. The public log corrects a premature status update from a shell sequence that continued after the audit failed. Applying parent visibility after child updates fixed the scene; `pi-perception-test-002` passed all 112 image audits using the same prescribed seed and conditions. Invalid captures and predictions remain locally for diagnosis.

## Recorded results

The frozen checkpoint is SHA-256 `60d8a832914e6e67ff2db5c91e6b702cefe87057c96c0850bb940e755aa0ca31`, selected at epoch 39 with validation foreground mean IoU 0.8741. Its freeze timestamp precedes the final test capture. All 240 development and 112 test RGB file hashes are distinct, with no overlap between these sets.

| Class | Ordinary IoU, 80 images | Challenge IoU, 32 images |
| --- | ---: | ---: |
| Cable | 0.9769 | 0.9680 |
| Contacts | 0.6475 | 0.6012 |
| Stiffener | 0.9569 | 0.9439 |
| CSI | 0.8912 | 0.8829 |
| DSI | 0.8935 | 0.8961 |

CSI has false positives in two of four absent-object views. This small denominator and weak contact segmentation are reasons to continue perception development, not enable insertion. Median CPU inference is 119.1 ms and p95 is 126.5 ms under the timing definition below. Two isolated replays produced identical masks and object outputs for all 112 images.

The website includes all 112 RGB frames with lossless prediction and annotation overlays, the result tables and a 23-second review video. Browser checks cover desktop/mobile and light/dark themes, 42 view combinations per configuration, populated tables and video metadata. The live page adds camera placements, a proposed physical sensor shortlist and the DINOv3/SAM 3.1/DINOv2 comparison plan. Those stronger models are researched candidates, not installed or measured replacements. Software validation: 70 tests pass, Ruff passes, and the diff has no whitespace errors.

## Metrics and limits

Per-class IoU pools pixels across each reported subset. Foreground mean IoU averages the five task classes and excludes background. An entirely absent class with no prediction is reported as unavailable rather than an artificial perfect overlap.

Object review takes the largest four-connected region of at least eight pixels. A centroid within 12 pixels is a descriptive localization statistic, not an insertion tolerance. False detections on absent objects are counted separately. Individual contact fingers may be smaller than the component threshold, so use contact pixel IoU rather than interpreting the component statistic as contact-end detection.

CPU timing includes network inference and mask/probability extraction, excluding image decode, connected-component review and file output. A second independently launched isolated process checks exact equality of masks and object outputs; timing is intentionally excluded.

Before robot motion we still need real-camera calibration and holdouts, endpoint/face and mouth-plane estimates, calibrated uncertainty and abstention, measured contact geometry and tactile/force feedback. Before insertion learning we also need cable deformation, friction, latch response, seating and electrical checks.

## Reproduce

First fetch and convert the E009 assets. Keep original and converted CAD outside the public repository. Install CAD tooling in a separate Python 3.12 environment using `requirements-cad.txt`. The footprint and lettering JSON are committed, so regeneration is optional.

```bash
python3 scripts/fetch_raspberry_pi_cad.py
.venv-cad/bin/python scripts/convert_step_to_usd.py third_party/raspberry_pi/pi4-detailed.step third_party/raspberry_pi/pi4-v2.usdc --split-connectors
.venv-cad/bin/python scripts/convert_step_to_usd.py third_party/raspberry_pi/camera3/Camera_module_3_std_model_simple.stp third_party/raspberry_pi/camera3.usdc

docker build -f Dockerfile.perception -t ffc-perception:2.8.0-cu128 .
# Isaac image and FR3 assets are prerequisites; see ISAAC.md.
scripts/run_pi_perception_experiment.sh your-fresh-run-name
```

The runner builds a fresh review scene, captures/audits development data, trains, records the frozen model hash, captures/audits the independent test scenes, runs isolated inference twice, compares replays and scores the predictions. It leaves public website publication to a separate review step. Use a new run name and preserve failed outputs.

The perception image uses the pinned Python 3.12 bookworm base digest, PyTorch 2.8.0 CUDA 12.8, NumPy 2.2.6, Pillow 11.3.0 and SciPy 1.16.1. CAD tooling is separate from Isaac Python. The runtime was tested on the RTX 5080 (16 GB). The [official PyTorch version matrix](https://pytorch.org/get-started/previous-versions/) documents the CUDA 12.8 wheel for this release.

The live page is a read-only observation service with an explicit temporary public connection. See [LAB_LIVE.md](../LAB_LIVE.md) for status publication, camera capture, service lifecycle and the limits of the browser-local notes field.
