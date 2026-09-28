# Motion command foundation: dry-run only

The first motion component is `src/ffc/motion_proposal.py`. It creates a bounded joint-space proposal from two normalized image errors and a **measured** local image-to-joint Jacobian. No actuator, ROS publisher, simulator object pose or scene query is imported. Every output has `executable: false`. This is preparation for non-contact visual servoing, not a moving-robot demonstration.

## Contract

Inputs are acquisition times, measured joint positions/velocities, normalized image error, the corresponding measured 2 × 7 Jacobian, reviewed joint corridor bounds, and a combined calibration/model/Jacobian identity. Both image rows must use the same normalization in the error and Jacobian. The measured Jacobian is not yet available for our mounted cell; the tests use an explicitly synthetic fixture.

The planner applies a half-gain pseudoinverse step with a single scalar limiter. A stop-to-stop quintic trajectory determines the maximum increment from its peak velocity and acceleration, as well as an absolute per-joint step cap. Defaults are provisional software-test limits: 0.2 s duration, 0.001 rad maximum increment, 0.01 rad/s velocity and 0.05 rad/s² acceleration. These values are not identified FR3 dynamics or validated lab limits. Jerk, torque, payload, collision and drive-tracking constraints remain unqualified.

The corridor is an input that must be independently qualified. Merely checking joint bounds does not prove collision clearance. The intended adapter must reject an expired proposal before accepting it and enforce the full trajectory and stop behavior after acceptance. `valid_until` is a dispatch deadline, not a braking guarantee.

Only one proposal may be pending. Completion requires matching sequence identity, fresh joint feedback acquired after issuance, elapsed trajectory duration, target tracking and measured settling. The next proposal requires a camera acquisition after that settling measurement. Camera age is limited to 500 ms and joint feedback age to 100 ms. Changed identities, missing vision, nonfinite data, unsettled joints and rank-deficient image responses are rejected. Explicit stop and backwards clock jumps latch a stop. Reinitialization is a reviewed operation, not automatic recovery.

The current pixel geometry is not an approved visual control target. A low fit residual does not imply reliable localization. Two image features also do not observe all robot degrees of freedom; the pseudoinverse is a local proposal, not a general safe controller.

## Local verification

Thirteen tests cover step/velocity/acceleration bounds, expected error reduction in a synthetic linear image model, one-step sequencing, post-settle reacquisition, stale/future sensor data, NaNs, abstention, identity mismatch, deficient Jacobians, measured movement, corridor exit, invalid completion, trajectory duration and stop/clock-reset latching.

The current restricted session cannot access `/var/run/docker.sock`, so Isaac and ROS integration were not run. It also cannot create a local HTTP test socket. The first full test invocation therefore exposed the environment restriction in `test_lab_live.py`; that existing test was not weakened. A completion-test timing fixture also needed correction after adding the duration check. The final non-network suite passes **125 tests** with only `test_lab_live.py` explicitly excluded. This is not a claim that the full suite or robot integration passed in this session.

## Commission motion in this order

1. **Unloaded FR3 free-space adapter.** Use the actual articulation and measured joint state. Verify units, joint ordering, bounded trajectory tracking, cancel, timeout and stop. Do not teleport links or reconnect the historical privileged-state task controller.
2. **Independent free-space corridor check.** Include robot links, full tool, stand, table and workpiece fixture. Test intermediate trajectory samples and conservative bounds, not just endpoints. Audit swept clearance separately from the visual policy. Review cable/grasp assumptions before including a held workpiece.
3. **Measure local visual response.** In the qualified region, run small reversible probes and estimate the image Jacobian from camera features and measured joint increments. Record probe direction, settle time, fit residuals and conditioning. Check on separate probes. Do not compute this mapping from perfect simulator cable/board poses.
4. **Shadow proposals.** Feed fresh deployed observations to this planner without dispatching. Verify identity continuity, uncertainty/visibility gating, deadlines and proposed corridor membership. Establish an explicit reviewed visual goal from deployable observations.
5. **Bounded stop-and-observe alignment.** Dispatch one qualified increment, measure completion, acquire another image, and replan. Test wrong-sign response, stale frames, tracking loss, motion cancellation and calibration changes. Remain clear of contact.
6. **Contact is a separate milestone.** Deformable cable/grasp mechanics, force/tactile signals, damage limits, insertion/latch behavior and recovery must be qualified before insertion.

The renderer's roughly 1.2 fps rate favors discrete stop-and-observe experiments. It has not been qualified for continuous visual servo control. No simulation motion was enabled by this change.
