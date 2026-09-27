# Raspberry Pi CAD import and two-task hardware scope

The selected first task is a Pi 4 Model B with Camera Module 3 Standard and the official Standard–Standard 200 mm camera cable. The second task is a phone-style FPC with a press-on board connector. Stage 2 is scoped, not yet modeled or trained.

## What is implemented

- Pi 4 community STEP from [MGS-CAD-Files](https://github.com/multigamesystem/MGS-CAD-Files), commit `c2cc26be009e7c2825ff7b8162e8863d88b8be08`.
- [Official Camera Module 3 simplified STEP](https://pip.raspberrypi.com/categories/1207-design-files), normal FoV.
- Open CASCADE STEP import, assembly placement, CAD colours, named component metadata, USD tessellation in metres. Linear deflection 0.015 mm; angular deflection 0.15 rad. These are tessellator settings, not claims of source accuracy.
- A dimensioned, static FFC visual asset with opposite exposed faces, 15 separate contacts per end, reinforced ends, and an authored 200 mm centreline curve.
- Dedicated `/World/Hardware` namespace inside the existing FR3 workcell. The generic PCB, slot, old cable and regrasp fixture are removed from this review scene. Historical baseline configuration is preserved separately.
- Seven actual Isaac Sim 6.1 RTX views and a 72-frame inspection orbit. No robot motion or physics advancement. The saved stage is `outputs/pi4-cad-review-002/raspberry-pi-workcell.usda` locally.
- Source hash lock, dimension audit, static rendering report and public hardware review page.

The two CAD assemblies contain 216 imported leaf components and 779,957 triangles. A leaf can contain several solids. No task collision geometry, latch articulation, electrical contact model or calibrated cable mechanics is implied by this count.

## Dimension evidence

The CAD audit extracts cylindrical faces from the Pi PCB solid. Its 85 × 56 mm outline and four 2.7 mm mounting holes agree with the [official mechanical drawing](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-mechanical-drawing.pdf). Hole centres measured from the PCB lower-left are (3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5) mm. Source PCB thickness is 1.6 mm; that thickness is not independently certified by this outline drawing.

The [official cable drawing RP-008146-DS-1](https://pip-assets.raspberrypi.com/categories/786-raspberry-pi-camera-module-3/documents/RP-008146-DS-1-standard-camera-cable-200mm-reference-drawing.pdf) provides length 200 mm, width 16 mm, pitch 1 mm, conductor width 0.7 mm, conductor thickness 0.027 mm, strip length 5 mm, support length 6.3 mm, header thickness 0.309 mm and insulation thickness 0.055 mm. The body thickness 0.137 mm is derived as conductor plus two film layers; physical adhesive and reinforcement transitions remain to be checked. The drawing shows opposite exposed faces.

The Pi 4 CSI socket is upright. Its insertion approach is normal to the board, not along the old horizontal generic slot. A 5 mm exposed conductor is not an insertion-depth specification. Follow the [official installation instructions](https://www.raspberrypi.com/documentation/accessories/camera.html#install-a-raspberry-pi-camera) and measure the actual seat and latch travel.

## Fidelity status

**Not training-ready.** Pi 4 small passives and board silkscreen are missing. The connector mesh does not separate materials by physical subpart. CAD colours and documented material overrides are not calibrated BRDFs. Camera distortion, noise and lighting require real measurements. The camera module lens is geometry only, not an optical simulation.

The official Camera Module 3 STEP is itself simplified. In particular, the source assigns a misleading generic component name to the optical assembly; the conversion preserves source labels rather than claiming a corrected BOM.

The current cable is a visual mesh, not a deformable solver. No old 8 mm cable stiffness, mass, friction or success threshold is transferred to this 16 mm cable. There are no task colliders on the imported hardware. This avoids an accidental convex hull across the socket but does not supply a validated collision model. Static review has `motion_permitted=false` and `training_ready=false`.

No explicit asset licence was found in the MGS repository. Original and converted vendor/community CAD are kept in ignored `third_party/`; this repository publishes import code, provenance, reports and review renders. Alternatives inspected: PartCAD's colourless Pi 4 STEP and hannahdesigns' CC BY CAD Crowd model whose STEP download requires an account.

## Stage 2: phone-style press-on FPC

A representative research coupon is preferable to an unspecified phone spare part for initial characterization. [Hirose BM28](https://www.hirose.com/en/product/document?clcode=&documentid=en_BM28_CAT&documenttype=Catalog&lang=en&productname=&series=BM28) is a candidate: 0.35 mm pitch, 0.6 mm mating height, documented alignment guidance. This is not a claim that a particular phone uses BM28. The catalog specifies 10 mating durability cycles, so connector wear and replacement must be part of physical trial accounting.

Next steps for this separate task:

1. Select a matched plug/receptacle and obtain both CAD and handling specifications. Design replaceable board and reinforced FPC coupons with accessible electrical test points.
2. Define the flex's copper/polyimide/adhesive stack and 90° routing radius. A routing turn does not change the connector's normal mating direction. Include residual bend and peel load.
3. Build a fixture that supports the PCB near the socket. Define a grip and press surface on the reinforced area from supplier guidance; do not press on unsupported flex.
4. Record synchronized close-up images, tactile contact and fixture/wrist force during alignment, parallel mating, one-sided engagement, release and disconnection. Identify limits and transient force signatures on sacrificial samples.
5. Fit a connector model against held-out traces. Keep snap retention, spring compliance and flex deformation distinct. Do not implement seating as a hidden-distance-triggered snap-to-pose shortcut.
6. Train local sensor-driven alignment and pressing from a stable grasp before introducing desk pickup and routing. Stage transitions and rewards use deployable signals. A tactile click alone is insufficient evidence of full seating.
7. Require even seating, post-release retention and continuity/short checks. Report failed contacts, damage, interventions and specimen wear.

FR3 motion interfaces, sensor recording and evaluation structure carry over. Connector mechanics, tactile signatures, gripping geometry and acceptance criteria do not automatically transfer.

## Reproduction

```bash
python3 scripts/fetch_raspberry_pi_cad.py
uv venv --python 3.12 .venv-cad
uv pip install --python .venv-cad/bin/python -r requirements-cad.txt
.venv-cad/bin/python scripts/convert_step_to_usd.py third_party/raspberry_pi/pi4-detailed.step third_party/raspberry_pi/pi4.usdc
.venv-cad/bin/python scripts/convert_step_to_usd.py third_party/raspberry_pi/camera3/Camera_module_3_std_model_simple.stp third_party/raspberry_pi/camera3.usdc
.venv-cad/bin/python scripts/audit_raspberry_pi_cad.py
mkdir -p outputs/pi4-cad-review-new
docker compose run --rm experiment scripts/render_raspberry_pi.py --output /workspace/outputs/pi4-cad-review-new
```

Use a fresh render directory. Verify `review.json` exists and records seven views, 72 orbit frames and zero elapsed timeline time. Source downloads must match `config/raspberry-pi-sources.json`; a mismatch stops the fetch instead of silently replacing the asset.

Validation: 65 Python tests pass, including metre-scale cable width, 15-contact pitch, opposite supported faces, 0.309 mm tip thickness and curve arc length. The separate CAD audit passes board outline and hole checks. These checks do not certify contact or appearance fidelity.
