# Live mounted-camera inference and presentation demo

The frozen mounted-camera DINOv3 model now receives processed observations over ROS and publishes `FeatureFrame` messages. This is the first live connection of that evaluated model to the native simulated camera. It remains segmentation on a stationary cell, with motion disabled.

## Admission and output

The installed predictor verifies checkpoint/backbone hashes, checkpoint metadata and the reviewed fixed camera contract before loading. It warms the GPU before subscribing. The inference gate requires the direct `world → macro_optical_frame` static transform to match the frozen rig within 1e-6 per transform entry, along with matching native calibration identity and transformed intrinsics. That tolerance is numerical identity checking, not a mechanical mounting tolerance. A changed TF latches a fault until a reviewed restart.

Incoming observations undergo full envelope validation and a 500 ms acquisition deadline. The same checks run again after prediction and message construction. Late results are discarded. Each published feature carries the original acquisition header, calibration, model SHA-256, rig fingerprint, class order and processing duration. Mean softmax scores remain uncalibrated. No pose or actuator command is emitted.

`mounted_predictor.py` owns GPU model execution; `inference_gate.py` owns sensor admission/completion; `inference_node.py` owns ROS conversion and publication. `compose.inference.yaml` mounts only reviewed calibration, frozen model files and backbone weights into the runtime. Scene USD, object transforms and offline labels are absent. Fixed camera TF is deployable calibration metadata, not privileged object state.

The static TF check validates agreement between declared calibration and the model contract. It does not measure a physical camera or authenticate arbitrary ROS publishers. The current single-threaded executor checks the latest transform it has received; a future moving camera needs timestamped dynamic TF synchronization and stronger clock/reset handling.

## Tests

The installed ROS test suite passes **19 tests**, including rig identity, quaternion sign equivalence, altered processed intrinsics, post-inference expiry and recovery. Core CPU tests also pass.

`outputs/ros2-inference-002` runs actual DDS messages and the frozen GPU model in an isolated domain. Its accepted prediction is pixel-identical to the original offline prediction for `0000-macro.png`. It rejects missing TF, stale input, wrong profile/native calibration, truncated edges and self-consistent but wrong K. Fresh valid input recovers. Dropout produces expired-feature diagnostics. A one-millimetre TF translation change rejects subsequent input; restoring TF does not clear the latched fault. Two valid feature frames were emitted; eight invalid observations were rejected. All motion flags remained false.

`outputs/ros2-inference-slow-001` wraps the actual model with a test-only 600 ms delay. The valid input becomes stale during execution; **zero feature frames** are emitted. No production delay parameter was added.

The first DDS attempt, `ros2-inference-001`, failed because its input filename omitted the `-macro` suffix. No test passed in that attempt. The corrected run used the existing input; the empty directory Docker had created for the nonexistent bind source was removed. Failed logs are retained.

## Native live recording

`outputs/live-feature-demo-001` contains **47 matched observation/feature pairs**, spanning **38.826 seconds** of acquisition time. Median predictor processing time was **47.234 ms**; frame age at viewer receipt was **90.283–105.845 ms**. This measurement includes the actual model output copy and score aggregation, so it is not the earlier encoder/head-only timing.

The viewer joins source image and mask by acquisition timestamp, then writes one atomic JSON snapshot with the encoded review image and its metadata. The server exposes it through a read-only `/api/perception` endpoint. Human visualization has no control endpoint. Display age is client-clock based; the authoritative inference deadline is enforced in ROS.

The fallback video preserves the recorded acquisition intervals and adds a short final hold. It is a static-scene recording, not robot motion. Its provenance is in `docs/demo/recording.json`. There is no audio track; presenter narration is supplied in [the runbook](../DEMO.md) and on [the demo page](https://lakshya-asu.github.io/ffc-insertion-research/demo/).

## Remaining gates

This connection does not calibrate prediction uncertainty, infer a qualified 3D entrance frame, or validate deformable contact. A metric estimator must still identify visible features, abstain on ambiguity, quantify error against independent references and reject expired measurements. Then cable/contact mechanics and sensor-based control can be qualified. Static TF and 1.2 fps rendering are not sufficient for dynamic robot control.

## Reproduce

Build `Dockerfile.ros2`, then run `bash scripts/run_inference_checks.sh outputs/ros2-inference-checks-NEW` with a fresh destination. This runs the normal and intentionally delayed DDS tests without networking or offline annotation mounts. Use `bash scripts/start_demo.sh` and `python3 scripts/check_demo.py` for the native live pipeline. The viewer/recorder entrypoint is `scripts/observe_live_features.py`; it accepts `--record-dir` only for a fresh directory.
