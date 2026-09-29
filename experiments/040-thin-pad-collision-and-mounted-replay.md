# 040 — Thin-pad collisions and mounted-motion replay

29 September 2026. Follow-on to the failed mounted lift in 039.

## What the diagnosis found

Run 004 added an independent, offline body-pose audit before control begins and sampled tool-body poses during the run. Its intended tool pose matches the physical tool to submicrometre precision. The cable begins straight. The CAD pad surfaces are about 1 mm apart, yet the mesh-collider run reports pad contact before closure reaches the cable. This is a collision representation problem, not a reason to weaken contact acceptance.

The installed Isaac 6.1.0 PhysX schema declares `physxConvexHullCollision:minThickness = 0.001` metres. The CAD pads are 0.3 mm thick. Run 005 explicitly lowered this floor to 10 micrometres; its observed trace remained identical to 003. Therefore changing this setting did **not** establish the cause or solve the problem. Mesh cooking/caching remains a hypothesis, not a proven explanation.

Run 006 uses an analytic box for each rectangular pad, derived from the CAD vertices in the rigid-body frame. Export checks that the pad vertices lie on the box corners before using this representation. The visible CAD remains; its duplicate mesh collider is disabled. The rest of the physical tool still collides. Screw clearance envelopes remain excluded. No contact threshold or friction value was relaxed.

The early run-006 trace has zero pad load at the open 1 mm gap. Near closure, both loads rise to roughly 0.15 N and the controller begins lifting. Final qualification is recorded below when the trial finishes.

## Data and replay improvements

- Initial offline audit: measured tool poses, declared expected tool transform and cable initial poses.
- Recorded tool-body poses beside cable poses, at the inspection frame cadence. These remain offline only.
- Native FR3 replay: reconstructed robot links from recorded joint encoders; tool and cable bodies from recorded poses. Physics is disabled in the packaged USDZ.
- Reopened run-005 replay agrees with every recorded arm FK, tool translation and cable translation check to below 1e-8 numerical tolerance. Start and end render successfully in Isaac 6.1.0.
- Mounted sensor episode schema includes measured arm angles and issued references/gravity efforts, with camera absence and unknown action-application times explicit. Offline body poses are not included.
- A frozen source/asset rerun bundle builder checks source snapshots against their hashes and CAD against the recorded report. Bundles include the pinned-base runtime build recipe, a fresh-output launcher and file hashes. A fresh runtime image build is not yet tested; installed-runtime rerun results are recorded separately.

## Commands

```bash
uv run python scripts/archive_fr3_replay.py outputs/fr3-flex-006 outputs/fr3-replay-006
uv run python scripts/bundle_mounted_run.py outputs/fr3-flex-006 outputs/frozen-fr3-flex-006
```

Both require completed trial outputs and fresh destination directories. Replay export additionally requires the offline tool-body capture introduced in run 004. A replay is sampled recorded motion; a rerun executes physics again.

## Remaining limits

Presented cable, ideal simulated contact/encoder feedback, approximate masses/inertias, uncalibrated material/friction/damping, and no camera servo, ROS motion bridge, suction or insertion. Whole-tool and robot/environment contact audits do not yet constitute a complete self-collision or swept-clearance proof. Force qualification is not independently verified cable retention. Positive lift, empty jaws and feedback dropout must all be evaluated on the same frozen candidate before closing the mounted-motion gate.

## Completed nominal suite

Run 006 kept the grasp but plateaued at 9.5799 mm and failed the existing tracking timeout. Run 007 disabled articulation sleeping but reproduced the plateau. Sleep disabling alone therefore did not fix the motion. The vendor joints retain their authored friction setting of 0.2.

Run 008 adds encoder-error integration at 5/s with a ±0.003 rad correction bound and position-limit anti-windup. It uses the existing `bounded_integral` implementation; it does not alter the cable, friction or acceptance thresholds. On fault, the adapter latches measured arm angles, clears the correction and commands zero velocity while preserving the jaw hold reference.

| Frozen run | Observed result |
|---|---|
| `fr3-flex-008` | Contact hold complete; measured lift 10.0318 mm; final lower/upper pad load about 0.1515/0.1486 N |
| `fr3-flex-empty-002` | Empty/thin grasp rejected; no commanded lift |
| `fr3-flex-dropout-002` | Loss detected; target latched to measured angles; additional tool lift 1.65 µm and joint drift 3.31 µrad over 0.25 s |

The independent suite reviewer checks identical frozen sources, tool asset identities, timestep and servo gain across these three runs. All 13 controller/source checks pass. The positive offline material point rises about 10.0316 mm; its maximum displacement relative to the tool after contact qualification is about 0.459 µm. This is a sampled ideal-simulation observation, not measured real-world precision or a damage bound.

Native replay checks cover 74 sampled cable/tool frames and every recorded robot encoder sample. The positive replay start/end were rendered again in Isaac. An isolated rerun from the failed run-005 bundle exactly reproduced all 115 command, joint-angle and pad-load samples on the installed image. The positive bundle is packaged but has not yet been independently rerun; rebuilding the runtime image from scratch also remains untested.

Run the candidate explicitly with `--joint-integral-gain 5`; zero remains the runner's baseline default for comparisons. The supplied positive bundle launcher preserves this setting.

Next: calibrated held-end images and leading-edge/visibility review, alongside broader placement, curvature, numerical and sensor-fault trials. Pi insertion remains disabled. The nominal suite does not establish a deployment success rate.

Validation: 173 CPU tests pass; script/core lint passes; the library has no browser JavaScript errors or horizontal overflow at 390 px, and all local artifact links and manifest hashes were checked.
