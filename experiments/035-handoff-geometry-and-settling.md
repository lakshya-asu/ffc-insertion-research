# 035 — Side-entry handoff and fixture settling

The next Step 2 component is built and reviewed: a side-entry presentation-fixture concept, a required-travel envelope for an integrated suction pickup, and four Isaac gravity/contact trials. No pickup or insertion is demonstrated. All gripper options remain open.

## Geometry

The PGEA/finger asset is unchanged (SHA-256 `d1528020b71255129e236dcacddfbadc502632b19ea413d55d2ebdffbda8350e`). Its orientation changes: the housing stays beside the ribbon, rather than along the cable tail. An inline housing/cable AABB comparison flags three possible overlaps. The side-entry fixed-housing/cable lower-bound separation is 9.250 mm. These bounds do not establish exact intersection in the rejected orientation.

The pad footprint is now **3 mm across, 6 mm along the cable**, so experiment 034's original orientation cannot be reused without rotating the footprint. At a center setback of 12 mm, with ±0.5 mm independent placement error, ±10° yaw and 1 mm margin, the pad envelope lies from 8.285 to 15.715 mm behind the leading edge. It clears the assumed 4 mm contacts and 6 mm stiffener. Both opposing pads are checked through the conservative full-width exclusion rule. These are still dimensional assumptions.

The fixture supports the body 30 mm above the desk, starting 20 mm behind the tip. A second support carries the tail. The connected CAD solid is exported as STEP and STL; fasteners, radii, material and manufacture remain open. Swept per-mesh AABBs enclose continuous linear approach and jaw motion. All checked meshes clear the desk and fixture, with a minimum lower bound of 3.000 mm. This is not a robot-path, bracket, hose or nozzle-mount clearance qualification.

The integrated alternative places a 7 × 19 mm cylindrical cup envelope at a 26 mm setback. In this particular layout it needs 30 mm of relative vertical travel between desk acquisition and the jaw plane. No slide actuator or mount is selected; the envelope is not supplier CAD. Fixture delivery is also not tested. These branches therefore remain alternatives, not finished handoffs.

`outputs/handoff-review-001` preserves the first layout. Revision 002 adds the housing/tail comparison and hides the fixture's pickup envelope after hypothetical delivery. The 72-frame, six-second Isaac review in `outputs/handoff-render-001` advances zero physics steps. The fixture cable stays fixed; the integrated cup moves empty while its cable stays on the desk. The video states those limits.

## Gravity/contact trials

`scripts/isaac_fixture_settle.py` builds an unanchored 200 mm ribbon from 100 rigid segments and D6 joints. The first 6 mm uses the assumed 0.3 mm header thickness; the rest uses 0.14 mm. Width is uniformly 11.5 mm: far-end widening and its stiffener are omitted. Visible contacts occupy the first 4 mm on the lower face. No gripper, attachment joint, vacuum or controller is present.

The homogeneous equivalent model assumes density 1800 kg/m³ and nominal E = 3 GPa. Body EI is 7.889e-6 N m². Adjacent half-segment rotational compliances add in series. Angular drive stiffness is converted from per radian to per degree as documented by [OpenUSD](https://openusd.org/dev/api/class_usd_physics_drive_a_p_i.html). Equal swing gains follow [NVIDIA's D6 guidance](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.0/dev_guide/rigid_bodies_articulations/joints.html); torsion and in-plane bending are not independently identified. Friction, damping and solver settings are assumptions, not laminate identification.

All four runs simulate two seconds, with PGS, 255 position and 8 velocity iterations. Measurements below are segment-center drops, not a verified leading-edge pose or equilibrium result.

| Run | E scale | Timestep | Leading segment drop | Grip-region segment drop |
| --- | --- | --- | --- | --- |
| fixture-settle-001 | 1 | 0.25 ms | 4.689 mm | 1.342 mm |
| fixture-settle-002 | 0.25 | 0.25 ms | 4.872 mm | 1.375 mm |
| fixture-settle-003 | 1 | 0.125 ms | 2.508 mm | 0.724 mm |
| fixture-settle-004 | 4 | 0.25 ms | 4.577 mm | 1.304 mm |

**Material behavior is unqualified.** Halving the timestep changes nominal tip drop by 2.181 mm, approximately 46.5%. This exceeds the 0.295 mm spread from the tested stiffness range at the coarser timestep. The solver/contact formulation is materially affecting the answer. Do not tune E to hide this difference, select a grasp height from these numbers, or describe the movies as validated cable mechanics. This repeats the class of numerical concern found in experiment 005 for a different cable.

## Evidence and next gate

Each run preserves its source, offline positions, report, stage and video. The source-only unused-import correction after run 001 changes no mechanics. All four results are published together by `scripts/publish_handoff_settling.py`. The local hardware page has the geometry video, fixture CAD, settling comparison and all four physics videos. Desktop/mobile page checks show no horizontal overflow; the geometry video loads at 1600 × 700 for six seconds. CPU suite: 148 tests passed; targeted lint passed.

Next isolate a short cantilever with the new thickness/segment sizes and compare its static deflection with an independent beam/discrete-chain reference. Check timestep and discretization convergence before returning to fixture contact. If the segmented solver cannot meet a declared error budget at a practical timestep, compare a shell or rod formulation. Then test fixture placement, suction release, camera-observed end position and bounded physical pinching. The visual clearance result alone does not advance Step 2 to complete.

```bash
/tmp/ffc-tool-cad-env/bin/python design/handoff/build_review.py --output outputs/handoff-review-NEW
docker compose run --rm experiment design/handoff/render_isaac.py \
  --stage /workspace/outputs/handoff-review-002/handoff.usda --output /workspace/outputs/handoff-render-NEW
docker compose run --rm experiment scripts/isaac_fixture_settle.py \
  --output /workspace/outputs/fixture-settle-NEW --stiffness-scale 1 --dt 0.000125
```
