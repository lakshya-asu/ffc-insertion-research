# Zero camera-only reliability pass

This pass targets the previous model's weak mini-end localization and adds an explicit abstention decision. It remains a static synthetic component-mask experiment. No mouth plane, latch state, physical leading edge, insertion pose or robot action is inferred. Every decision denies motion permission.

## Design fixed before the test

`config/zero-reliability-v2.json` records the sample sizes, seeds, loss, checkpoint criterion, gate rules and threshold grid. Development has 200 scenes (160 train, 40 validation); threshold calibration has 80 new scenes; the final test has 120 new scenes. Each scene has an entrance and offset camera view. Conditions are sampled independently from a fixed categorical mixture: ordinary 45%; absent board 6%; absent cable 6%; empty 4%; dim 6%; bright 6%; large offset 7%; large yaw 7%; tool occlusion 8%; digital blur 5%. Board/cable pose, presentation and lighting are independently sampled within the declared bounds. Both camera views of a scene stay together.

The native image and fixed local crop contract is unchanged from experiment 014. No simulator location chooses the crop. The camera views are simulated B0498/16 mm reference projections, not calibrated physical lenses. Blur uses a Gaussian radius of two pixels after rendering; it is a digital stress case, not a validated defocus model.

## Correcting the capture pipeline

The first development attempt `outputs/zero-reliability-dev-001` failed closed at scene 170. An absent-cable frame contained eight pixels labelled mini end, on an unrelated part behind the connector. The raw RGB showed no cable. Its `invalid.json`, debug RGB/mask/mapping and source snapshot are preserved. It is excluded from training and selection.

Run 002 retained persistent objects but was also rejected at scene 32: the semantic-ID lookup assigned broad cable regions to the mini-end class. Persistence alone did not solve the problem. The replacement capture uses the renderer’s `instance_id_segmentation` output and maps visible instance IDs to explicit USD mesh paths offline. It bypasses the dynamic semantic-class lookup rather than deleting unexpected labels. Board connector paths are resolved through the CAD component metadata; cable, mini-end and jaw paths are explicit. Initial overlays are inspected in a separate 40-scene audit before the full run. An independent offline containment audit projects a conservative mini-end envelope from authored poses and verifies that every mini-end label lies within it. It passed 105,662 pixels across the 80 audit images and rejected 36,875 out-of-envelope pixels in a preserved failed annotation. This geometric check is supervision QA only; its poses and projected boxes are never inference inputs. Strict absent-object checks remain; both failed runs are excluded. Historical capture implementations remain available in git and source snapshots. The precise internal renderer cause has not been established. Retrospective containment checks also pass the previously published development set (160 images, 212,900 mini-end pixels) and test set (72 images, 67,305 mini-end pixels). Containment does not establish pixel-perfect completeness or physical geometry accuracy.

## Model and objective

The available official frozen DINOv2 ViT-B/14 encoder supplies the same features as the earlier model. Two independently initialized task heads use seeds 928611 and 928612. Both train for 60 epochs on the new development set. The loss computes cross entropy separately for each image/class and averages class contributions, with twice the weight for the mini end. A per-image foreground Dice term penalizes mask mismatch and absent-class predictions. This prevents large visible ends from dominating the small ones. Each member's checkpoint is selected by mean per-visible-image mini-end validation IoU (visibility: at least 16 ground-truth pixels).

The ensemble averages the two softmax maps. It shares an encoder and training data, so its errors are correlated. This is not a full independent-backbone ensemble, and disagreement is not a calibrated probability. The previous frozen DINOv2 model is run on exactly the same fresh test images to separate improvement from changes in test difficulty. No claim isolates the effects of more data, new loss and ensembling individually.

## Review gate

The runtime worker uses only RGB-derived masks and probabilities. For connector and mini end, it computes agreement between the two heads and mean predicted-class softmax; the minimum across these four values is a ranking score. It rejects missing/small components (64 pixels connector, 32 pixels mini end), any target-class mask touching the crop boundary, black/white/constant frames, and extremely low image detail. The detail statistic is variance of the grayscale Laplacian after sigma-one smoothing; the 0.02 floor was chosen from development-image probes before calibration or test. It is not a calibrated optical-focus test.

A separate offline calibrator chooses a threshold for each camera from 0.30 to 0.95 in 0.05 increments. It maximizes accepted calibration examples subject to zero observed bad accepts and at least 20 accepts. If no threshold meets those conditions, that camera always abstains. A good component pair requires both classes to be visible and each full class mask to have IoU at least 0.75. Calibration success is selection evidence, not a reliability estimate.

The frozen policy binds checkpoint and backbone hashes, camera calibration IDs and all four inference-module hashes. Calibration IDs are recomputed from the supplied calibration content before use. The final inference worker has a read-only filesystem, disabled network, no CAD/scene mount and no offline annotation mount. It validates image hashes and the camera packet boundary. It uses a recorded acquisition clock for replay; real-time camera-driver health is not demonstrated. Accepted output is named `provisional_component_review`; rejected output is `need_another_view`. Both leave insertion pose/latch unknown and motion disabled.

## Evaluation

Report raw mask IoU and per-visible-image end IoU alongside coverage, accepted mistakes and retained good views. Report each camera separately because two views of one scene are correlated. Per-camera one-sided 97.5% Clopper–Pearson error upper bounds give at least 95% joint coverage for two cameras only under independent, identically distributed draws from this synthetic generator. They say nothing about other distributions or physical lab performance. No accepted samples means error is unknown. Test thresholds are not retuned after inspecting failures.

Fixed fault replays take the highest-ranked RGB-derived view from each camera (eligibility first, then gate score) and replace it with black, white or severe Gaussian blur (radius 12). Six diagnostic cases cannot estimate a fault-detection rate. Unit tests separately check missing regions, clipped regions, disagreement, non-finite scores, unqualified thresholds, and the absence of motion authority. Existing camera-ingress tests cover stale/repeated/reordered/malformed packets, simulator-state extra fields and calibration mismatch. A static scene with genuinely new timestamps may have identical pixels; image equality alone cannot identify frozen hardware.

## Research basis and limits

The [selective classification formulation](https://proceedings.neurips.cc/paper/2017/hash/4a8423d5e91fda00bb7e46540e2b0cf1-Abstract.html) motivates reporting the tradeoff between accepted coverage and error. The [deep ensembles paper](https://proceedings.neurips.cc/paper/2017/hash/9ef2ed4b7fd2c810847ffa5fa85bce38-Abstract.html) motivates comparing independently initialized predictors. This implementation is a task-specific empirical gate; it does not reproduce either paper's guarantees or claim calibrated uncertainty from agreement alone.

Neither model is trained on real camera images. Geometry remains a community CAD assembly with unverified aperture/latch detail. Cable shapes and jaw boxes are static visibility probes, without measured flexural mechanics or contact. The physical entrance, leading edge, lens focus, mounts, real gripper occlusion and tactile/force sensing remain the next qualifications. A component review must not be promoted into an insertion target.

## Reproduction

Use fresh output paths. Stop the separate live preview before captures. Source/weight locks and Docker images are as in experiments 013–014. Validation must succeed before training. Run the calibration capture only after model freeze; capture the test only after both models and gate are frozen.

```bash
set -e
docker compose run --rm experiment scripts/capture_zero_perception.py \
  --profile reliability --split development --output /workspace/outputs/NEW-dev
.venv/bin/python scripts/validate_zero_dataset.py outputs/NEW-dev
mkdir outputs/NEW-model
docker run --rm --gpus all --network none --user "$(id -u):$(id -g)" \
  -e PYTHONPATH=/code -e XFORMERS_DISABLED=1 \
  -v "$PWD/outputs/NEW-dev:/data:ro" -v "$PWD/outputs/NEW-model:/results:rw" \
  -v "$PWD/third_party/dinov2:/backbone:ro" \
  -v "$PWD/scripts/train_zero_reliable.py:/code/train_zero_reliable.py:ro" \
  -v "$PWD/src/ffc/zero_region_model.py:/code/zero_region_model.py:ro" \
  ffc-perception:2.8.0-cu128 /code/train_zero_reliable.py --data /data --output /results
docker compose run --rm experiment scripts/capture_zero_perception.py \
  --profile reliability --split calibration --output /workspace/outputs/NEW-calibration
.venv/bin/python scripts/validate_zero_dataset.py outputs/NEW-calibration --development outputs/NEW-dev
scripts/run_zero_reliable.sh outputs/NEW-calibration/sensor outputs/NEW-model \
  outputs/zero-region-model-001 outputs/NEW-calibration-predictions
mkdir outputs/NEW-policy
PYTHONPATH=src .venv/bin/python scripts/evaluate_zero_reliability.py calibrate \
  outputs/NEW-calibration outputs/NEW-calibration-predictions outputs/NEW-policy/policy.json
docker compose run --rm experiment scripts/capture_zero_perception.py \
  --profile reliability --split test --output /workspace/outputs/NEW-test
.venv/bin/python scripts/validate_zero_dataset.py outputs/NEW-test --development outputs/NEW-dev
scripts/run_zero_reliable.sh outputs/NEW-test/sensor outputs/NEW-model \
  outputs/zero-region-model-001 outputs/NEW-predictions outputs/NEW-policy
PYTHONPATH=src .venv/bin/python scripts/evaluate_zero_reliability.py evaluate \
  outputs/NEW-test outputs/NEW-predictions outputs/NEW-evaluation.json --policy outputs/NEW-policy/policy.json
.venv/bin/python scripts/publish_zero_reliability.py outputs/NEW-test outputs/NEW-predictions \
  outputs/NEW-evaluation.json docs/live/NEW-review
```

## Frozen test outcome

On 120 independently sampled scenes (240 camera images), the two-head model reached
93.74% pooled foreground mean IoU versus 91.16% for the earlier DINOv2 model on the
same images. Mini-end pooled IoU was 88.87% versus 80.92%; mean per-visible-image
mini-end IoU was 75.26% versus 64.04%. Of 213 visible mini ends, 181 versus 145
exceeded 0.5 IoU. These comparisons combine changed training data, loss and heads;
they do not isolate the effect of ensembling.

Neither camera qualified during calibration: no frozen candidate threshold had
zero bad accepts and at least 20 accepted views. Both thresholds are therefore
null. All 240 test views abstain, including 122 whose component pairs meet the
offline mask criterion. Zero accepted errors at zero coverage establishes no
useful selective accuracy. Fault probes also abstain under this always-abstain
policy; they do not demonstrate selective fault detection.

No test-driven threshold adjustment was made. Improving the component masks did
not establish reliable acceptance, entrance geometry, stable grasp or insertion.
All 960 RGB/overlay assets were decoded at desktop and mobile widths, with no
JavaScript errors or page overflow. The 92-test repository suite passed.
