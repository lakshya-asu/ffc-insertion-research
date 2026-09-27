# Sensor-driven assembly: gated implementation

The historical Isaac controller uses true cable poses for alignment and stage checks. It is a geometry baseline, not a deployable skill. `scripts/run_pipeline.sh` now requires `--historical-baseline` to reproduce it deliberately. No new historical simulation was run for this change. Lower-level experiment scripts remain historical tools and must not be used as a sensor-driven benchmark.

## Milestone 0: observation ingress prototype

Implemented in `src/ffc/observations.py`; tested in `tests/test_observations.py`.

The vector ingress accepts only configured measured channels, checks dimensions and finite values, requires episode-local timestamps, rejects stale/future/out-of-order data and excessive cross-sensor skew, and creates immutable copies. Rejected packets do not advance its state. Reusing an unchanged low-rate reading is allowed only while fresh. A new episode requires a new gate.

This is **not yet wired to Isaac or hardware**. It has no camera-buffer transport, calibration registry, sensor forward model, controller or learned policy. It does not certify sensor authenticity: someone could put true poses into a permitted vector. Isolation of producers, restricted process messages, source review, and leakage tests are still required. Accepted input is not permission to move. Thresholds in unit tests are synthetic test values, not physical operating limits.

Run `.venv/bin/pytest -q` to reproduce the software tests. At this revision 42 tests pass (17 existing, 25 new ingress cases). No physical performance claim follows from that result.

## Milestone gates

| Gate | Build | Evidence required to proceed |
|---|---|---|
| M0 | Typed vector ingress | Fault injection passes; no simulator imports in the ingress. Current limited prototype complete. |
| M1 | Fixed physical camera mounts; image/DAQ adapters; policy process isolated from simulation | Raw timestamped recordings, calibration IDs, camera dropout and delayed packet tests, no truth keys or USD access in policy process. Replay identical inputs with changed hidden truth and verify identical outputs. |
| M2 | Exact cable/connector/PCB characterization | Part drawings, measured bend/twist/indentation and insertion curves, held-out validation with uncertainty; mesh/time-step convergence separately reported. |
| M3 | Desk perception | Held-out cable-face/end/grasp-patch labels; pixel and physical error budgets, calibrated confidence and explicit rejection of occlusion. No movement while unresolved. |
| M4 | Suction pickup and tactile handoff | Pressure traces, tactile/slip calibration, jaw-load envelope, successful vent while retaining cable; failures and retries included. Compare direct and passive fixture under matched conditions. |
| M5 | Local alignment and insertion | Sensor-only policy and transitions, signed force response, bounded retreat/reobserve behavior, off-axis jams and partial insertions evaluated. |
| M6 | Latch, withdrawal and electrical verification | Defined latch mechanism, supported closure, accessible far-end test circuit, false-positive checks with known faulty assemblies. |
| M7 | Imitation learning and optional residual RL | Episode-disjoint dataset, sensor-conditioned actor and critic, held-out evaluation against sensor-driven baseline. No privileged reward shortcut. |

Every gate produces an immutable run manifest, raw observations/actions, software/calibration hashes, stage video and an evaluation report. A missing measurement is `not evaluated`, never a pass. Videos show what happened; numerical measurements decide whether the gate passed.

## Physical inputs still needed

Exact cable and connector samples, PCB mounting and test circuit, actual camera/lens geometry, tactile finger fit and readings, gripper force and vacuum measurements. Simulation development can proceed to M1 without these. Hardware-calibrated M2 and physical qualification cannot be completed from renders or literature alone.

## Next executable increment

M1 begins with one stationary cable and fixed global/local cameras. Save clean images without debug labels and camera intrinsics/extrinsics. Add image timestamps to ingress and test missing/frozen frames. Implement cable/end detection and evaluate on annotated images before connecting any action output. Camera placement may depend on known robot kinematics for a rigid wrist mount, never on hidden cable-tip position.
