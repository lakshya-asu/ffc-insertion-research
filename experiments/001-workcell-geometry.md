**E001 — FR3 workcell geometry, before contact physics**

Date: 2026-09-26. Status: executed; endpoint results only. Protocol specified before execution.

Question: Can a single FR3 reach representative desk-pickup and horizontal connector-approach poses with a suction/pinch tool envelope, while avoiding modeled obstacles at those endpoints?

Hypothesis: an elevated, edge-accessible connector fixture permits horizontal approach while a deployed downward suction nozzle permits pickup from the desk. A fixture flush to the desk may constrain the tool body even when the flange pose is kinematically reachable.

Protocol: deterministic damped-least-squares pose IK, fixed seeds, bounded joint positions, multiple initializations. Evaluate suction-contact and lifted-contact poses at five desk positions, plus connector approach at three distances. Compare a 120 mm and a 5 mm support height. Each target has a fixed orientation. Test endpoints only; do not infer collision-free paths or dynamics from endpoint feasibility. Report position/orientation residuals, actual qpos, and all modeled penetrations. Resolve orientation with a rotation-vector error, not Euler subtraction.

Position convergence criterion: 0.2 mm. Orientation convergence criterion: 0.5 degrees. These are numerical solver thresholds, not required insertion tolerances or measured physical accuracy. Reject modeled penetration exceeding 0.1 mm, except the robot base/table mounting interface. Preserve smaller contacts in diagnostic output.

Robot: pinned upstream Menagerie FR3; stock position actuators are retained but no dynamic manipulation is evaluated. Tool, cable and connector: geometric proxies. Cable remains a static thin strip. Suction, grasping, bending, friction, latch operation, electrical checks and force response are not simulated. No insertion-success metric is permitted from this experiment.

Decision: use the results to refine fixture height, tool envelope, and target locations. Keep physically validated deformable mechanics as a separate subsequent milestone.

Compute: CPU IK and one headless render; minutes expected. Source revision, runtime versions, seed, configuration checksum, and results are recorded in outputs/e001/.

**Observed results.**

| Fixture support height | Pickup/lift endpoints found feasible | Connector approaches found feasible | Total |
|---|---:|---:|---:|
| 120 mm | 9/10 | 3/3 | 12/13 |
| 5 mm | 9/10 | 0/3 | 9/13 |

These are a fixed set of pose checks, not randomized assembly trials or success-rate estimates. Twelve starts were allowed per endpoint. A numerically converged endpoint was accepted only if its modeled contact penetrations met the stated criterion.

The reported raised-layout failure was `pickup_1_lift`: position residual 0.000146 m and orientation residual approximately 0.00061 rad, with rejected robot self-intersections. No collision-free converged endpoint was found within the fixed search budget. This is not a proof of physical unreachability.

The low-layout failures were `pickup_1_lift`, `approach_0.020m`, `approach_0.010m`, and `approach_0.003m`. Each low connector approach converged geometrically but had rejected desk intersections, including the tool body and suction nozzle. All detailed distances and joint configurations remain in results.json; these failures were not excluded from totals.

Decision: use the 120 mm fixture as the provisional layout for tool development, while retaining the low-fixture case as a regression and exploring alternate tool geometry. Resolve the failed pickup-lift posture with collision-aware IK/path planning or a justified workspace revision before freezing the acquisition region. Do not claim continuous pickup-to-insertion feasibility from these endpoints.

Validation: seven tests pass, covering home joint limits, a 180-degree rotation error, the rotational Jacobian convention, replay of an accepted endpoint, rejection of an unreachable target, rejection of colliding converged IK, and fixed-seed reproducibility. Ruff check passes. The first lint pass failed on import placement, loop-variable naming and line length; these were corrected before the clean check. Network asset retrieval was exercised in setup but is not part of offline unit tests.

Robot source: google-deepmind/mujoco_menagerie at c96a32d28fb5da84da38c1da4d749e7a13212855. Runtime: Python 3.14.6, MuJoCo 3.12.0, NumPy 2.5.3. See config/fr3-source.json for per-file checksums and upstream license path.
