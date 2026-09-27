# E011 — Task-driven sensor placement study

The original desk/board cameras were chosen for static semantic segmentation. They neither supplied adequate cable-end sampling nor established visibility with the gripper in the approach path. This study designs a separate candidate layout around pickup, inspection and downward insertion into the Pi 4 CSI connector.

The configuration is `config/sensor-layout-v2.json`. It specifies four fixed ideal 2448 × 2048 cameras: overhead workspace coverage, an end-presentation station and opposing oblique CSI views. Camera positions are optical centers, not mount bolt locations or lens-front working distances. A 450 mm overhead field gives 5.44 pixels/mm; 45 mm close-up fields give 54.4 pixels/mm on a plane normal to the optical axis at the target. This is sampling, not localization accuracy. Lens MTF, focus, exposure, distortion and calibration errors are not modeled.

## Why separate the observations?

The overhead image finds the loose cable and supports approach planning. It is not expected to resolve insertion clearance. After lifting, the robot would deliberately present the reinforced end to the fixed inspection view, then reorient to inspect both faces. This is a proposed skill with no passive regrasp fixture requirement. Fixture-assisted handling remains a separate option to compare.

Two board cameras look across the upright ribbon from opposite sides, about 28 degrees above the board. They leave the central downward approach open and provide different views of occluded edges. Opposite faces do not automatically share corresponding features, so these are not an off-the-shelf stereo-depth pair. The eventual estimator must fuse calibrated views and observable physical edge geometry, with uncertainty and abstention.

## Rendered study and its limits

Four discrete situations are authored: loose cable, end presentation, tip 15 mm above nominal socket height and tip 3 mm above nominal socket height. Each is viewed by all four cameras. Cable poses are rigid transforms of a straightened display ribbon; gravity, retained grasp, flexible deformation, collision and insertion are not simulated. The normal cable length and contact/stiffener geometry are retained. No commanded FR3 motion occurs, and the timeline remains paused.

Two jaw boxes represent a possible clearance envelope: 4 mm thickness, 12 mm width, 3 mm height, center 5 mm behind the tip, 0.4 mm inner gap. That gap is deliberately not claimed to be a closed, load-bearing grasp on the 0.309 mm header. The boxes do not represent the existing FR3 tool, a tactile sensor package or a selected actuator. The source CAD does not provide a qualified open-latch geometry or a calibrated socket mouth frame. The nominal review center cannot be sent to a controller as an insertion target.

The original static benchmark and its model remain unchanged. The new optics are not evidence of improved learned-model performance. The live renderer remains on the original cameras; the new study is explicitly separated on the same live page.

The first completed optical run (`sensor-layout-v2-005`) exposed weak contrast at the end-presentation view. The revised run adds a matte dark backdrop 100 mm behind the presentation plane and a diffuse area light facing that region. Both are authored design candidates: radiometry, lens response and physical mount clearance remain unqualified. A before-lighting image is retained for comparison. Earlier setup failures are preserved in run directories 001–004; they are not successful optical studies.

## Nonvisual sensor placement

- Route every Pi support through one plate on a six-axis force/torque transducer. Prevent parallel mechanical load paths. Strain-relieve cables and measure their residual force contribution. Overload stops must not carry normal working loads.
- Place tactile contact on a pinch surface behind the free insertion edge. A full-size DIGIT package may not fit the short stiffener and narrow approach. Package a thin finger around measured samples before selecting the tactile hardware.
- Put the vacuum pressure/flow measurement near the suction tip, with short plumbing. A seal is not enough to prove stable cable retention; combine it with observed lift and tactile evidence after regrasp.
- Retain measured gripper position and FR3 telemetry. These complement fixture force sensing; they do not prove seating or electrical connection.

None of these physical sensors is installed or calibrated.

## Gates before choosing mounts or training control

1. Add selected camera/lens bodies, brackets, actual fingers, tactile package, actuator and cable routing. Check FR3 reach and swept-volume collision throughout pickup, presentation, approach and retreat. Current optical-center coordinates alone do not pass this gate.
2. Sweep grip depth, cable bow, lateral error, yaw and board pose. Measure edge visibility with the real tool, deliberate occlusions and uncertain detections. Require another view or stop when the relevant geometry is unobservable. Four authored cases are only an initial visual review.
3. Calibrate real intrinsics/extrinsics, robot-frame alignment and timestamps. Use controlled diffuse lighting and compare polarization where glare is a problem. Measure useful resolution, focus and repeatability over the working volume. A provisional 0.10 mm edge-localization development target is not an insertion tolerance; derive that from connector measurements.
4. Validate tactile retention, fixture noise, overload behavior and sensor latency independently. Train the stronger perception model on the revised image contract only after those observations have been reviewed; test on fresh synthetic and real holdouts.

## Reproduce

With the E010 refined scene and Isaac container available:

```bash
docker compose run --rm experiment scripts/review_sensor_layout.py --output /workspace/outputs/your-new-sensor-study
.venv/bin/python scripts/publish_sensor_review.py outputs/your-new-sensor-study docs/live/sensor-study
```

The publisher checks all 16 image dimensions and exact pixel equality after lossless WebP conversion, calculates ideal axis sampling, and creates four labelled montage frames. Encode those at one frame per four seconds for the stage-review video. Publish only after checking every relevant camera view and the browser.

## Sources

- [Edmund Optics: imaging system parameters](https://www.edmundoptics.com/knowledge-center/application-notes/imaging/6-fundamental-parameters-of-an-imaging-system/) — field of view, working distance and resolution definitions.
- [Basler camera portfolio](https://downloads-ctf.baslerweb.com/dg51pdwahxgw/1jFygRZWsQshfx9yU3lWuQ/ac2844162de4d6c1666d8fc5e532a117/BAS2501_CVP_No86_A4_EN_SAP0024_web__1_.pdf) — candidate 2448 × 2048, 2.74 µm-pixel, global-shutter sensor format. No purchase or exact lens selection is implied.

## Entrance-specific correction

The user correctly identified a missing task feature: we must observe the entrance, not merely the connector body. `config/csi-entrance-audit-v1.json` compares an axial view with west/east views offset about 11° from the board normal, with 28 mm fields. These are alternative axes to compare before committing additional cameras.

`audit_csi_recess.py` reads the community connector triangles and samples their highest surface at 25 µm XY spacing. It identifies a connected low region under a declared z=36.8 mm plane, 0.10 mm below the housing lip. The region includes end recesses and is not a qualified usable mouth. Its sample centroid is about 0.52 mm away in X from the previous nominal review center. That is a CAD diagnostic, not real localization accuracy. Physical part identity, open-latch geometry, rim definitions and available clearance must be verified before treating these as entrance-training labels.

Raw RGB is from `csi-entrance-audit-001`, with no annotation mesh. A separate `csi-entrance-audit-005` render pass holds a labelled cross-section mesh visible and uses the renderer’s occlusion to produce offline masks. No label-pass RGB is published as raw camera input. Configurations match, all timelines remain paused, and the empty presentation control has identical mask pixel counts to the unobstructed case. Runs 002–004 exposed stale semantic mappings when toggling the annotation mesh; their zero labels on an unobstructed control were rejected. Separate passes avoid that visibility-toggle failure.

| Camera | Reference mask pixels | 15 mm-above probe visible | 3 mm-above probe visible |
| --- | ---: | ---: | ---: |
| Axial | 99,917 | 0.0% | 1.6% |
| 11° west | 97,117 | 48.1% | 26.4% |
| 11° east | 97,758 | 22.8% | 8.1% |

The cable/jaw probes retain the old nominal review center deliberately, exposing the weakness of a whole-connector target. These are four static diagnostic situations, not aligned trajectories or a robust visibility sweep. Straight-down imaging is useful before approach but insufficient once occluded. A near-axial macro view must be complemented by another observable view and a tool whose sightlines have been designed and tested. The perception contract now separates visible aperture, rims, latch, cable tip and occlusion; no entrance estimator has been trained.

## Existing camera inventory

The user reports owning Arducams capable of roughly 4K at 15 fps; exact model, quantity and lens are unknown. Official specifications identify B0471/B0471C (IMX519 autofocus USB3) and B0498 (IMX585, manual C-mount) as matches to 3840 × 2160 at 15 fps. B0433 instead offers 3840 × 3032 at 15 fps MJPEG and 3840 × 2160 at 30 fps MJPEG. These are candidates, not ownership identification. Evaluate existing cameras before buying the earlier Basler shortlist.

Read-only USB/sysfs enumeration on 2026-09-27 identified a RealSense D405 at 480 Mb/s and D415 at 5000 Mb/s; no attached USB camera identified itself as Arducam. No physical images were captured. This does not inventory cameras attached to another computer or a CSI interface. Physical camera placement and calibration are unknown. D405/D415 can be considered for coarse geometry, but no slot-depth accuracy is established.

A native 3840-pixel width covering 28 mm gives an ideal 137 px/mm. The lens must actually resolve and focus that field at a usable distance. At 15 fps, 66.7 ms frame spacing precedes additional sensor, transport and inference delays. Begin with stationary inspection and measured move–settle–observe behavior, not an assumed 15 Hz safe control loop.

The generated CAD cross-section stays local in `outputs/csi-recess-audit.json`; the public repository contains the extraction code and rendered evidence, not that derived geometry. Regenerate it before a label pass:

```bash
.venv/bin/python scripts/audit_csi_recess.py
docker compose run --rm experiment scripts/review_sensor_layout.py --config /workspace/config/csi-entrance-audit-v1.json --output /workspace/outputs/new-entrance-rgb
docker compose run --rm experiment scripts/review_sensor_layout.py --config /workspace/config/csi-entrance-audit-v1.json --recess-labels --output /workspace/outputs/new-entrance-labels
.venv/bin/python scripts/publish_entrance_review.py --rgb-run outputs/new-entrance-rgb --label-run outputs/new-entrance-labels --web docs/live/entrance-study
```

The `--recess-labels` pass contains an annotation mesh and must never supply RGB to a learner. Its published overlays combine masks with the independently captured raw RGB run.
