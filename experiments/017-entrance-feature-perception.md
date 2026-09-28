# Four simulation steps: CAD audit, feature labels, visibility, learned perception

This experiment advances Step 1 entirely in simulation. Real-camera validation is
deferred. It does not claim the complete physics-based pickup/insertion task is
solved, and it does not reuse the historical privileged-state assembly controller.

## 1. Source geometry audit

The connector named Molex 54548-2271 in the community Zero 2 W CAD is one connected
mesh after 10 nm vertex welding, with no separate actuator or joint. Its board-local
bounds are X 28.3–33.3 mm, Y −8.9–7.3 mm, Z 1.6–2.8 mm. A center-section drawing
is retained. A component outline is insufficient to certify the entrance plane.
The [manufacturer page](https://www.molex.com/en-us/products/part-detail/545482271)
identifies a slider, 22 bottom contacts, 0.5 mm pitch and 1.2 mm mated height.
These specifications do not identify the installed Pi BOM or provide slot tolerances.

For this experiment, the fused connector alone is replaced by an engineered
simulation substitute within its envelope. The rest of the community PCB stays.
The opening width (11.7 mm), height (0.5 mm), slider travel (0.8 mm), internal
contact geometry and all clearances are assumptions in `config/zero-feature-v1.json`.
The sliding part is visually positioned; it does not yet clamp or actuate contacts.
This geometry makes feature-level simulation executable without pretending the
community CAD provides manufacturing internals.

## 2. Feature supervision

Instance IDs map to physical upper/lower front-rim surfaces, a 0.3 mm-long band
at the beginning of the existing blue stiffener, and the slider in authored
extended/retracted states. All labelled meshes use their normal materials. No
semantic colors or markers enter RGB. The narrow band approximates a leading-edge
neighborhood, not the zero-width edge itself. It cannot establish a 3D tip frame.

An initial audit found a retracted slider buried in the housing; that geometry
was rejected before training. The corrected slider is above the housing roof and
inside the original height envelope. A geometric regression test checks this and
the clear rim-to-rim opening. Dataset audits check state consistency, class presence,
absent-object labels, image hashes and split separation. Occluded labels remain
empty; no hidden feature is drawn through an occluder.

## 3. Visibility and stress

A preceding 39-configuration sweep compares two camera views and finger dimensions
against matching no-finger baselines. Full component pixel retention is a screening
measure, not aperture observability. The feature dataset uses 120 development
scenes, 96 training and 24 validation, each seen by both cameras. Conditions cycle
through ordinary, specular-light probe, dim, rotation, finger occlusion, blur,
absent cable and absent socket, with independently randomized poses and slider state.

The reference cameras retain native 3840×2160 projection and a fixed 896×448 crop.
The actual user's Arducam model remains unidentified. Gaussian blur and dimming
are digital stresses, not calibrated exposure, MTF or readout models. Cable and
fingers remain static and rigid for this perception experiment. The authored
lighting probe is not a measured photometric calibration.

## 4. RGB-only learned estimator

Official pretrained DINOv3 ViT-B/16 supplies frozen intermediate features. A
six-class decoder uses those features and native RGB detail. Training uses
per-image class-balanced cross entropy plus Dice loss. Validation chooses the
checkpoint by mean per-visible-image feature IoU; model selection never sees test
images. The shared decoder architecture does not make this a controlled DINOv2
versus DINOv3 comparison because geometry and classes changed.

After model freezing, a different seed generates 48 test scenes (96 views).
Inference runs without networking in a read-only container with only sensor PNGs,
weights and explicitly mounted runtime modules. It cannot read labels, meshes,
poses or the full repository. Offline scoring reports per-class IoU, a descriptive
2-pixel boundary F1, absent-object false positives, and slider-state errors. The
32-pixel minimum for a provisional slider prediction is fixed before testing;
it is not a validated latch or motion gate. No output permits motion.

## Physics work that remains

The existing native FEM benchmark already exposed timestep, mesh and iteration
sensitivity; the earlier segmented controller used privileged state and is not
acceptable as the new learned proof of concept. Next physics tests must establish
finite-thickness contact, cable bending/twist convergence, friction/slip and
bounded contact responses before connecting the image estimator to action.
Neither the new static dataset nor a smooth rendering substitutes for those tests.

Reproduce all stages in fresh directories with
`scripts/run_entrance_features.sh NEW_RUN_PREFIX` after installing the documented
Isaac and foundation images and approved local DINOv3 checkpoint.

## Frozen candidate and fresh final test

The first model's validation exposed poor thin-band performance. A second candidate used focal CE plus Dice, capped square-root inverse-frequency weights and 120 epochs. It was selected using development validation only, then frozen as `outputs/entrance-focal-model-001`. The changed epoch budget means this is not a controlled loss-only ablation. A fresh test seed 929971 produced 48 scenes / 96 images in `outputs/entrance-feature-final-test-001`, disjoint from development and the earlier test. Evaluation: `outputs/entrance-feature-final-evaluation-001.json`.

Pooled IoU: upper rim 89.2%, lower rim 88.7%, leading band 39.3%. Mean visible-image IoU: 85.4%, 78.0%, 32.3%, respectively. Provisional slider classification was correct in 52/81 visibly labelled images, unknown in 10 and wrong in 19. This is not an insertion-ready estimate. The page exposes all 96 RGB/prediction/offline-label triplets; the browser verified all 288 assets at desktop and mobile widths. Macro optics are a separate domain and need a new evaluation, not reuse of these scores.
