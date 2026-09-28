# Entrance observation and stable grasp: milestone contract

The milestone requires evidence that the cable end and real connector entrance
remain measurable while the tool holds and approaches the cable. A static visual
study can screen tool envelopes; it cannot establish grasp stability.

## First executable experiment

`scripts/render_zero_grasp_visibility.py` renders the existing Zero 2 W scene
with two reference cameras and 39 static configurations: three no-jaw baselines,
plus 36 combinations of tip-to-review-point distance (15, 3, 1 mm), jaw-center
setback (6, 10, 15 mm), jaw width (8, 12 mm), and thickness (1.5, 3 mm).
Jaw length is 4 mm and inner gap is 0.5 mm. The assumed cable header is 0.3 mm;
there is deliberately no claim of contact or a closed grasp. Cable and jaw
positions are authored offline, not produced by a controller.

Compare each camera with its own no-jaw baseline at the same cable distance.
Renderer instance IDs supply offline visible component pixel counts. The ratio
screens occlusion of the mini-end and connector body. It does not isolate the
entrance rims or cable leading edge. Inspect every RGB frame before selecting
candidate tool arrangements. Do not optimize component retention as if it were
insertion success. Increasing setback exposes more cable but may make it less
stable; this tradeoff requires mechanical measurement.

The source USD is opened with edits in its anonymous session layer and never
saved. Physics time must remain unchanged. Outputs include all RGB images,
offline masks, cases, counts and the source hash. Failed runs remain preserved.

## Later hardware validation (deferred; not a simulation blocker)

1. Identify the actual camera and lens; calibrate intrinsics, distortion,
   exposure and camera-to-fixture geometry at the intended focus setting.
2. Measure cable thickness, width, stiffener/exposed lengths, and connector
   entrance/latch geometry. Record measurement uncertainty and specimen identity.
3. Photograph the entrance rims and cable leading edge through a real finger
   approach. Include open/closed latch and empty/occluded entrance examples.
4. With a restrained coupon, measure slip onset versus normal grip load and
   unsupported length. Inspect for contact/stiffener damage. Derive test limits
   from specimens and hardware ratings rather than selecting arbitrary forces.
5. Compare direct pickup with fixture-assisted presentation under matched cable
   placements. Count pickup, orientation, slip and recoverable failures separately.
6. Validate cable bending and insertion force/displacement before using contact
   simulation to train action policies.

Acceptance thresholds for physical pose error and grip stability remain unset
until clearances and sensor resolution are measured. Runtime inputs are RGB,
calibration and measured tool/robot/contact signals. No mesh, instance mask or
simulated object transform enters a deployed estimator or policy.

## Foundation-model work

Official SAM 3.1 checkpoint access is approved and the 3.5 GB multiplex checkpoint
has downloaded. Source is pinned to
`2345a4ad109ac29c569da749c91d84f10dc08c40`; checkpoint repository revision is
`daa63191845a41281374e725f4c9e51c7a824460`. Dockerfile.sam3 isolates its dependencies.
Compatibility and task accuracy must be measured separately. The official
`build_sam3_predictor(version="sam3.1")` factory selects the multiplex architecture.
Use fixed text or image-derived prompts, never simulator boxes or points.
Independent static images must be separate sessions, not a fictitious video.

DINOv3 access remains gated at the last authenticated check. Its adapter passed
random-weight shape checks at 448 by 896 pixels; no pretrained result is claimed.
A future DINOv3 comparison should reuse fixed native crops and a matched decoder,
with fresh final test scenes after training and prompt decisions are frozen.

Primary implementation references:
- https://github.com/facebookresearch/sam3/blob/2345a4ad109ac29c569da749c91d84f10dc08c40/RELEASE_SAM3p1.md
- https://github.com/facebookresearch/sam3/blob/2345a4ad109ac29c569da749c91d84f10dc08c40/sam3/model_builder.py
- https://github.com/facebookresearch/dinov3

## Updated scope: simulation first

The user explicitly requests a full simulated proof of concept before hardware work.
All four perception steps therefore proceed in simulation, with independent
synthetic held-out scenes. Physical validation remains a later transfer milestone.
Simulated contact will require numerical convergence and mechanics references;
we will not substitute simulator poses for measured controller inputs.

DINOv3 access was subsequently approved and its official checkpoint downloaded.
The feature-level experiment uses pretrained DINOv3; the earlier shape-only status
above describes the preparation stage. SAM 3.1 GPU inference passed after adapting
the upstream unsupported false `offload_state_to_cpu` argument. The six initial
probes were empty cable/socket scenes, and returned empty masks; no localization
performance is inferred from them.
