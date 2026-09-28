# Camera and model interface hardening

The active perception path now shares a canonical camera/model contract, and the ROS supervisor validates the entire processed observation before reporting it healthy. This is a software-integrity milestone. Motion remains disabled and the live learned-model adapter is still uncommissioned.

## Changes and ownership

See [Interfaces and code ownership](../IO_CONTRACTS.md) for the channel map, clock/frame conventions, file boundaries and module responsibilities.

`src/ffc/macro_contract.py` owns camera dimensions, class order, preprocessing identity, immutable calibration parsing, fixed-rig fingerprinting, checkpoint metadata and the discrete label transform. The ROS package includes the same file through a symlink; Docker copies its target before building the package. Training and scoring share that label transform instead of maintaining independent resize code.

Offline inference now verifies both frozen checkpoint and backbone digests before loading, then compares the full camera calibration to the frozen sensor contract. The output includes the fixed-rig fingerprint. No new model was trained or selected.

`observation_contract.py` checks the atomic ROS envelope: matching image/edge/calibration headers, complete payloads, exact dimensions and encodings, finite intrinsics, rectification and projection consistency, quality fields and version/profile identities. The supervisor also applies timestamp and calibration-continuity guards. Invalid input clears its accepted observation; recovery requires a valid fresh message. Status never authorizes motion.

## Validation and retained failures

- Core CPU suite: **106 passed**.
- Middleware-free observation tests: **10 passed**, also added to GitHub CI.
- Installed ROS container suite: **15 passed**, including OpenCV label-transform parity and malformed-envelope rejection.
- DDS pipeline `outputs/ros2-pipeline-008`: valid frame accepted with acquisition stamp preserved; stale, wrong-frame, changed-calibration, mismatched-stamp and blank inputs emitted no observation; dropout reported stale; a fresh frame recovered observation-only status. Motion stayed false.
- Isolated inference `outputs/macro-dinov3-predictions-003`: all **60 predicted PNGs byte-identical** to the original frozen test predictions.
- Offline rescore `outputs/macro-dinov3-evaluation-002.json`: class metrics, per-frame measurements, confusion matrix and slider outcomes exactly match the original report. Only the inference metadata/timing differs.

The first packaged integration attempt exposed an omitted public `REVISION` export. The pure frontend tests had not exercised that import. `macro-dinov3-predictions-002` and `ros2-pipeline-007` retain the failed attempt. The export was restored explicitly, a public-interface regression test added, the Docker image rebuilt, and the installed suite, full inference and DDS faults rerun successfully. Unit tests alone would not have caught this failure.

## What remains

The runtime intrinsic calibration identity does not lock camera extrinsics. A live inference adapter must bind the reviewed camera rig and frozen model, preserve acquisition identity, check age again after inference, and abstain on unsupported input. Those are the next integration gates. Metric pose uncertainty, cable/contact mechanics, force/tactile adapters and bounded control remain separate unfinished milestones.

The current image rate still produces legitimate stale intervals under the 500 ms deadline. This refactor does not change the deadline or claim closed-loop readiness. Historical scripts remain available for their recorded experiments; new work should use the shared contracts rather than duplicate their transformations.
