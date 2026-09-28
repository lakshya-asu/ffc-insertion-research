# Mounted-camera DINOv3: frozen model and fresh test

This milestone trains the first model specifically for the mounted 45° Basler/Kowa camera and the complete offset tool. The official DINOv3 encoder is frozen; a six-class feature head combines intermediate encoder features with RGB detail. This is static simulated segmentation, not a verified insertion pose or robot action policy.

## Protocol and data boundary

`config/macro-training-v1.json` declares seed 930301, 60 epochs, batch size two, the loss, validation selection criterion and independent test seed 930401 before testing. Development is the audited `macro-dataset-dev-001`: 128 train and 32 validation scenes. The exact ROS frontend `macro-rectify-area-half-pad4-v1` transforms native 2448 × 2048 RGB into 1232 × 1024. Labels use OpenCV INTER_NEAREST_EXACT at the same 2:1 reduction and side padding; calibration has zero distortion. A nonzero-distortion dataset requires matching label rectification and is rejected by this training/scoring path.

The feature head uses square-root inverse-frequency weighted focal cross entropy (gamma two, weights capped at 50), plus foreground Dice with double leading-band weight. DINOv3 blocks 6 and 12 provide frozen features. Head convolutions use bfloat16 autocast and the loss uses float32. The initial learning rate is 0.001 with cosine decay to 0.00003. The selected epoch is 59, chosen by mean per-visible-image foreground-class IoU on validation (at least eight label pixels). Selection reached 0.9183. Pooled IoU is reported separately and does not select the checkpoint.

The first launch (`macro-dinov3-model-001`) failed its fresh-output guard because source/protocol snapshots had been placed in its destination before training. It trained no model. The corrected run (`macro-dinov3-model-002`) completed 60 epochs in 134.19 seconds, including encoder feature extraction. Failed evidence remains intact. Source snapshots and the training protocol are archived with the completed model.

The checkpoint was frozen before capture of `macro-dinov3-test-001`: 60 new scenes, six per declared condition. Capture requires the frozen checkpoint and checks its hash. Native annotation integrity checks passed; test RGB hashes do not overlap development. Source geometry remains the same assumed socket and rigid visibility cable used in development.

Checkpoint SHA-256: `8668408a864e5a677d3b7b4896ceb5ea6ba13c0b934263a777c3dc7800ccf623`.

Frozen official DINOv3 backbone SHA-256: `9a21ac3df0c63839d62612dda6f454d816c25611cc7a52966ed5a5a94921dc8b`.

`run_mounted_inference.sh` mounts only sensor PNGs/calibration, weights and their metadata, explicit runtime modules, and a new output directory. Networking is disabled. It has no offline-label directory, object transforms, scene USD or repository mount. Camera calibration, profile, preprocessing revision, checkpoint hash and input dimensions are checked before inference. The separate scorer reads labels only after predictions have been produced. No test-driven model or threshold tuning was performed.

## Independent test outcome

| Class | Pooled IoU | Mean visible-image IoU | Boundary F1, 2 output pixels | Visible views | FP ≥8 pixels / below-visibility views |
|---|---:|---:|---:|---:|---:|
| Upper rim | 91.00% | 87.64% | 98.63% | 45 | 1 / 15 |
| Lower rim | 92.77% | 92.07% | 97.00% | 45 | 0 / 15 |
| Leading band | 92.85% | 92.88% | 98.45% | 42 | 0 / 18 |
| Open slider | 94.25% | 97.07% | 96.76% | 21 | 2 / 39 |
| Closed slider | 94.76% | 94.10% | 93.32% | 27 | 3 / 33 |

“Visible” means at least eight transformed ground-truth pixels. Its complement includes absent, occluded or tiny regions. The two-pixel boundary tolerance is an image-space description, not a calibrated insertion tolerance. Class scores are not confidence probabilities.

The provisional slider rule chooses the larger open/closed predicted area when at least 32 pixels are present; otherwise it returns unknown. It was inherited from the earlier feature evaluation and was not fitted to this test. Of 48 visibly labeled slider views, 47 were correct and one was wrong; none were unknown. All 12 unobservable-slider views returned unknown. Class-level false positive counts above and whole-slider classification measure different things.

The wrong slider case is `0043-macro.png`, a 6 mm short-grasp scene: authored closed, predicted open. The worst upper-rim IoU is 0.673 in focus-drift case 0046; the worst lower-rim IoU is 0.642 in near-entry case 0021. The worst visible leading-band IoU is 0.882 in board-rotation case 0012. The review exposes these failures alongside all other test images.

Median measured encoder-plus-head GPU inference was 26.0 ms; peak allocated CUDA memory was 1.01 GB. Timing excludes image decoding and the classical frontend and includes a GPU synchronization around computation. It is not full ROS latency or guaranteed sustained throughput. The static live renderer still runs near 1.2 fps. The new network has not been connected to a live ROS inference node or calibrated acceptance gate.

## What this result supports

The mounted-camera generator yields learnable rim and leading-band features with the full tool present. The leading-band result must not be presented as an isolated improvement over the earlier 39.3% Arducam-domain result: optics, labels in view, preprocessing, distribution and training budget differ.

The 60 scenes sample a narrow, balanced synthetic generator. They do not vary laminate deformation, material finish, lens MTF/noise, broad cable shapes or actual manufacturing dimensions. High overlap does not supply a 3D cable-tip frame, mouth plane, depth, stable grasp, qualified latch state or failure-detection rate. There is no calibrated confidence/abstention policy for this new model. Motion remains disabled.

Next: integrate the frozen estimator with observation timestamps/profile checks, retain inference age and stale-output rejection, and develop the relative-pose/uncertainty experiment. Before action, mechanics, contact/tactile signals and bounded recovery need their own qualification.

## Reproduction and public review

- `scripts/train_mounted_macro.py`: development training using the ROS image transform.
- `scripts/capture_mounted_macro.py --split test --frozen-model MODEL`: post-freeze capture.
- `scripts/run_mounted_inference.sh SENSOR_DIR MODEL_DIR NEW_OUTPUT`: isolated prediction.
- `scripts/evaluate_mounted_macro.py DATA PREDICTIONS NEW_REPORT`: offline scoring in the OpenCV environment.
- `scripts/publish_mounted_model.py`: curated review from the explicitly named runs.

Training and scoring run inside `ffc-ros2:jazzy-dinov3`; Isaac is used for capture. The public `/live/#macro-model` review includes all 60 RGB/prediction/offline-label triplets and a one-minute montage. Displayed RGB uses the same full field and padding; publication uses PIL area averaging, which may differ by one intensity level from OpenCV. Masks and scores use the exact OpenCV transform. No semantic colors are model inputs.

For a new training run, `scripts/run_mounted_training.sh VALIDATED_DATA_DIR NEW_MODEL_DIR` runs training in the tested container, then archives sources/calibration and writes the checkpoint freeze record. Do not prepopulate the model directory. Use `run_mounted_inference.sh` only with the sensor subdirectory, not the complete dataset. The native ROS preview should yield the GPU before capture/training and be restored afterwards.

All 180 public review images were loaded at 1440 px and 390 px widths with no JavaScript errors or horizontal overflow. All 60 prediction overlays were visually reviewed in contact sheets. The full montage is 1600 × 760, 60 seconds / 1,440 decoded frames. GitHub's CPU workflow passed the complete 97-test suite after adding the pinned public FR3 assets required by six geometry tests; GPU and ROS experiments remain separate evidence.
