# 046 — FR3 grip, lift and inspect in the Pi Zero cell

29 September 2026. The requested goal is sensor-driven pickup, orientation and insertion into the Zero 2 W. This milestone reaches a fresh presented-cable lift and native RGB inspection. It stops before orientation correction or approach because the visual alignment gate fails.

## Fresh physics result

`zero-cell-lift-001` runs the FR3 with the compact PGEA-2-10 candidate, full external tool collisions, a free 100-section cable, passive presentation support, Zero 2 W community CAD and the experiment-045 reference socket. The community connector mesh is disabled and replaced by the explicit open cavity, guides and 22 spring-supported contacts. This remains reference-family geometry, not a verified production socket model.

The jaws begin around the presented insulated body, 12 mm behind the mini-end tip. Ideal bilateral pad loads and arm/jaw encoders command closure, a 10 mm lift and holding. No cable pose setters run during the trial. Vacuum, camera-guided desk pickup and ROS motion transport are not connected.

- Final state: `contact_hold_complete`; measured lift **10.0318 mm**.
- Final pad loads: **0.1515 / 0.1486 N**, modelled signals, not damage thresholds.
- 24,041 physics steps at 125 µs; 3.005125 s simulated.
- Maximum arm speed 0.01440 rad/s; maximum joint reference error 0.003016 rad.
- No contacts crossed the independent unintended-contact audit threshold.
- Sampled tool/environment separation during this lift: 2.663 mm.
- Native video and 74-frame recorded-state USD replay archived. Replay transforms agree with recorded samples within 8.9e-16 numerically. Isaac start/end replay rendering passed.

The previous 70 × 38 mm board-support footprint obstructed the end of the nominal approach. A 60 × 26 mm support removes that overhang. A separate offline sweep of 12.5 mm translation and ±0.5° yaw over 153 samples gives a minimum component-bound separation of 1.116 mm. This is not global arm or camera-mount clearance and does not authorize motion.

## Inspection result: the gate fails

Two Basler/Kowa optical references view the terminal from opposite ends, above its plane. Native RGB is 1224 × 1024 with finite aperture. Camera calibration is ideal authoring calibration; no physical calibration accuracy is claimed. After holding, physics stops while eight zero-time rendering updates generate the images. The runner verifies that rendering did not advance the physics clock.

The estimator receives only RGB and projection matrices. Blue-component isolation, quadrilateral fitting and stereo triangulation return a surface leading-edge estimate. It does not consume USD, depth, segmentation annotations or cable transforms. The offline scorer independently reads recorded cable geometry afterward.

The earlier oblique camera layout abstained in all three development frames. A near-top layout measured 22–28 µm position errors in three adjacent recorded-lift frames. Those frames were used during development and are not a held-out reliability set.

The fresh physics run corrects the mini-end contact appearance to the underside. Its estimate has **95.3 µm position error**, including **−88.2 µm height bias**, despite a 0.927-pixel stereo reprojection residual. The assumed 0.40 mm throat and 0.30 mm terminal leave only 50 µm vertical clearance per side. The measurement fails that requirement.

Inspection suggests the cameras match different points on a finite-thickness silhouette. An experimental fit using the far top edge in each camera and known 6 × 11.5 mm terminal dimensions also abstains under its current consistency check. Its frozen failed result is retained. Do not use either estimator to authorize insertion. Face polarity is also not verified from observations; placing contact graphics downward is scene authoring, not perception evidence.

## Entrance views

Two static layout captures retain the actual held robot/tool geometry. The negative-side candidate is fully blocked by the tool. Two positive-side viewpoints can see the opening and contact noses. Depth of field still varies across the socket. These captures establish visibility only, not a metric entrance plane or an executed approach. The entrance camera in the fresh lift uses the earlier placement; its raw frame is retained separately.

## Reproduce and review

```bash
docker compose run --rm experiment scripts/isaac_fr3_flexible_lift.py \
  --output /workspace/outputs/zero-cell-lift-NEW --zero-cell --joint-integral-gain 5
```

For exact source inputs, use the archived `rerun-bundle.tar.gz`; the working tree can evolve. The bundle includes Zero CAD and camera dependencies. Its launcher syntax was checked; independent physics rerun and fresh Docker build are pending. The camera source includes the original estimator used by the physics run.

Review: [video, raw images, failures and downloads](https://lakshya-asu.github.io/ffc-insertion-research/library/zero-cell.html). Local captures and failed scores remain under `outputs/zero-*`. All motion/inspection evidence remains distinct from offline geometry labels.

## Next executable milestone

Resolve camera projection/edge-localization bias on explicit finite-thickness targets, freeze the estimator and test independent perturbed poses. Verify the visible contact-facing side. Fit and independently evaluate the socket entrance plane from the clear-side RGB views. Then connect those measurements to one bounded FR3 alignment correction, observe again, and only then connect the existing force-guarded insertion feed. Continue in simulation; no physical samples are required for this next stage.
