# Isaac joint motion and cancellation commissioning

Docker access was restored and the new joint-motion runner executed on the existing `ffc-isaac:6.1.0` image. Three successful trials use the official unloaded FR3 v2.1 articulation: a wrist probe/return, a shoulder probe/return, and a shoulder cancellation to hold. These are actual physics-driven movements, not authored link transforms or reconstructed animations.

The earlier E040 videos already demonstrated Isaac motion and geometric assembly in a different workcell with privileged cable/socket feedback. This milestone commissions a smaller joint-reference path without importing that task controller. It does not yet connect DINOv3 or the image-alignment estimator to robot motion.

## What runs

`scripts/isaac_joint_probe.py` creates a separate Z-up, metre-scale scene containing the vendor robot, a visual ground and two observation cameras. There is no cable, tool, Pi or connector. It initializes the robot once, then commands the articulation through position/velocity drives using measured joint positions and velocities. No pose setter is used after initialization.

The run reuses the earlier assumed Isaac drive gains and bounded integral correction, plus PGS with 255 position and four velocity iterations. Physics advances at 1 kHz. A pure quintic reference generator checks joint bounds, 0.00501 rad maximum step, 0.01 rad/s reference velocity and 0.02 rad/s² reference acceleration. Each probe requests 0.005 rad over two seconds, after an initial settling period.

The shoulder and cancellation runs additionally reject measured speed above 0.03 rad/s. The highest measured speed across each run, including initial settling, was 0.013831 rad/s. The reference bounds therefore must not be presented as measured-robot bounds. These are commissioning settings, not identified FR3 firmware or hardware operating limits.

A phase completes only after its duration and measured position error below 0.0002 rad on every joint, with measured speed below 0.001 rad/s. The runner rejects tracking deviations above 0.02 rad after the first half-second and settling that exceeds the declared phase deadline. Contact reporting and the existing independent experiment audit are enabled; none of these runs reported a contact pair. This is not a swept-clearance proof for the full tool or Pi cell.

## Results

| Trial | Run | Physics steps | Simulated time | Final maximum joint error | Video frames |
|---|---|---:|---:|---:|---:|
| Joint 7 probe and return | `isaac-joint-probe-002` | 7,003 | 7.003 s | 0.0000116 rad | 175 |
| Joint 2 probe and return | `isaac-joint-probe-003` | 7,003 | 7.003 s | 0.00000582 rad | 175 |
| Joint 2 cancellation | `isaac-joint-cancel-001` | 4,252 | 4.252 s | 0.0000161 rad | 106 |

The cancellation is requested halfway through the shoulder move. It holds the current commanded position and sets desired velocity to zero. Measured post-cancel joint excursion stayed below **0.00000716 rad**. The robot met the settling check after the prescribed 0.25 s hold, measured as 0.251 s including the final physics step. This is one simulated cancel-to-hold test, not a certified protective stop, braking-distance bound or fault-tolerant actuator design. The hold transition does not promise a jerk-limited stop.

All requested steps matched physics callbacks and elapsed simulation time within the clock-audit tolerance. Video capture uses zero physics delta; every capture checks that it did not change simulation time. Videos are recorded at 25 fps. The last 3 ms of the two full probes and 12 ms of the cancellation fall after the final scheduled video frame; joint/control auditing continues through the final physics step. Telemetry records joint samples at 50 Hz, while tracking and cancellation-drift maxima are monitored every control step.

## Failed run retained

`isaac-joint-probe-001` failed the settling deadline. It used the new runner's initial TGS setup and omitted an explicit stage up-axis. Final positions were near target, but joint 4 reported about 0.00749 rad/s, above the unchanged 0.001 rad/s settling threshold. The trace, failure report and video are retained.

The next run explicitly set Z-up/metres and used the PGS profile from the earlier successful work. It also improved lighting and added the contact audit. These changes were made together, so this experiment does not isolate which change caused the velocity difference. The first failure is not counted as a passing trial.

## Verification and reproducibility

132 CPU tests pass, including the restored local HTTP test and six new trajectory tests. Those numerical tests do not replace the three Isaac trials. The videos are available in the [demo motion section](https://lakshya-asu.github.io/ffc-insertion-research/demo/#motion), with measured/reference joint traces and public result JSON files.

Pause the separate live preview before another GPU simulation:

```bash
touch outputs/lab-live/stop-preview
# Wait for ffc-lab-preview.service to stop, then use a fresh name:
bash scripts/run_isaac_joint_probe.sh isaac-probe-NEW 7 probe
bash scripts/run_isaac_joint_probe.sh isaac-cancel-NEW 2 cancel
```

The wrapper checks `results.json` as well as the process exit. Isaac shutdown can obscure Python failures, so an exit code alone never establishes a pass. Each run archives source files and checksums. The successful wrist run predates the explicit measured-speed and cancellation additions; its own frozen source is retained.

The live perception preview was restored after the trials. No physical robot, ROS actuator endpoint, learned policy, pickup or insertion was enabled. Next comes the mounted-cell motion envelope and measured image-to-joint response, followed by bounded camera-driven alignment. The existing static Pi scene and tool/cable mechanics still need their own dynamic qualification.
