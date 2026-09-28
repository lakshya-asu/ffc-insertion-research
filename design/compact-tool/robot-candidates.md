# Available arms and the simulation comparison

The user confirms access to I2RT YAM and UFACTORY Lite 6. Exact YAM variant and controller revisions remain unspecified. Work remains simulation-only: prove conditional feasibility before hardware trials or purchases. FR3 remains the existing Isaac reference.

| Arm | Published specification | What it means here |
| --- | --- | --- |
| Lite 6 | 600 g payload; ±0.5 mm repeatability | Budget the whole wrist assembly. Evaluate visual corrections; taught-pose repeatability does not establish insertion accuracy. |
| I2RT YAM | 2 kg nominal payload; 750 mm maximum workspace range | More nominal payload headroom; fine positioning and contact performance remain unqualified. Confirm the available variant. |
| FR3 | Existing Isaac model and bounded empty-tool motion evidence | Keep the reference model. Existing motion evidence does not establish cable insertion. |

Sources checked 2026-09-28: [UFACTORY specifications](https://docs.lite6.ufactory.cc/9.technical_specifications.html), [I2RT product page](https://i2rt.com/products/yam-6-dof-arm), [I2RT documentation](https://doc.i2rt.com/products/yam). These describe products, not measurements of the lab specimens. This review did not establish a verified YAM endpoint-repeatability specification.

## Simulation qualification

Use identical connector, cable, fixture, cameras, initial-condition sets and scoring across arms. Score the seven skills separately: an arm may pass pickup without passing insertion.

1. Import the exact arm model into Isaac. Check flange frames, joint limits, workspace, self-collision and tool clearance. A supplier model is not calibrated dynamics.
2. Include adapter, fingers, sensors and routing in tool mass, center of mass and inertia. Verify allowed load offsets, not payload alone.
3. Sweep latency, low-speed tracking error, backlash, compliance, image age and calibration error. Label unmeasured values as sensitivity assumptions. Do not turn a repeatability specification into an invented Gaussian noise model.
4. Keep camera, measured-joint and tool/contact observations identical at the task interface. Simulator object truth is offline scoring data only. Do not expose ideal force measurements without a selected sensor model.
5. Measure stable correction size, residual entrance alignment, visibility, contact forces and stop/retract response. Derive tolerances from cable/connector geometry and uncertainty.
6. Evaluate held-out scenes and deliberate failures, reporting uncertainty and model limits. Simulation success establishes conditional feasibility, not physical qualification.

ROS 2 task interfaces should remain robot-independent. Robot adapters supply kinematics, supported commands, feedback and limits. No physical robot commands are part of this comparison.
