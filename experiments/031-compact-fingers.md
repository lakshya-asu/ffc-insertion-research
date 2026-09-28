# 031 — Compact PGEA finger geometry

## Result

Built a replaceable finger pair on the supplier PGEA side-exit assembly. The
assembled jaw groups pass 23 sampled solid-intersection checks. Four dimensional
cable coupons touch both pads without penetration. Isaac rendered 96 frames of
prescribed opening/closing motion, with **zero physics steps**. This establishes
nominal geometric fit, not a grasp or insertion result.

The desktop viewer now offers the compact assembly, finger close-up and existing
Pi workcell as separate views. The compact tool is not installed on the robot.

## Design

Each finger has a 10 × 9 × 3 mm mounting plate, a 6 mm wide, 1.4 mm thick blade
projecting 12 mm from the jaw face, and a 6 × 3 × 0.3 mm contact pad. The pad plane
extends 0.5 mm inward from the bare jaw plane. Thus a nominal 1–11 mm base opening
corresponds to a 0–10 mm pad opening.

One nominal M3 × 5 screw envelope passes through a 3.4 mm clearance hole. The
nominal 2 mm engagement stays within the supplier drawing's 3 mm threaded depth.
The screw has no modeled threads or socket. Its expected overlap with the
supplier tap-drill bore is recorded separately, not treated as a collision.

One locating-pin concept per finger uses the verified round supplier hole. Its
2.48 mm diameter and 2 mm insertion are preliminary; retention and actual fits
remain unresolved. The supplier's second, slotted datum is unused. A locating
pin is not represented as a proven press fit or friction-retained part.

Gold is a review color. No final finger or pad material is specified. Printed
parts would be fit prototypes until stiffness is evaluated. Edge radii, pad
attachment, screw preload, fatigue and cable pressure remain to be designed.

## Evidence

- `outputs/compact-fingers-001`: initial CAD and assembled sweep.
- `outputs/compact-fingers-002`: same geometry with explicit mounting-fit gates
  and four coupon contact checks.
- Both USD files have SHA256
  `d1528020b71255129e236dcacddfbadc502632b19ea413d55d2ebdffbda8350e`.
- `outputs/compact-fingers-render-001`: Isaac video, open/contact images, saved
  review stage and per-frame commanded jaw positions. Input is run 001, whose
  geometry is identical to run 002.
- Public review: [compact fingers](../docs/hardware/custom-gripper.html#compact-fingers).

The jaw sweep samples 1–11 mm every 0.5 mm, plus 1.14 and 1.30 mm. No opposing
group or fixed-housing intersection was found. Same-side fingers, pads and pins
do not overlap the supplier jaw group. The idealized screw thread envelope
intersects the tap-drill bore by approximately 4.004 mm³ per jaw; that is
intentional nominal engagement, not a fastening simulation.

The coupon checks cover both 11.5 and 16 mm widths at both assumed thicknesses,
0.14 and 0.30 mm. Pad-to-coupon distances are zero within numerical tolerance,
with zero finger or pad penetration. This says nothing about retention,
friction, cable damage or compliant deflection. The illustrated coupon extends
12 mm beyond the fingertip front and remains fixed throughout the video.

## Reproduce

Use the CadQuery environment described in `requirements-cad.txt`. Original
supplier CAD stays in ignored outputs; its source and hash are recorded in the
[bare-jaw audit](../design/compact-tool/geometry-audit.md).

```bash
/tmp/ffc-tool-cad-env/bin/python design/compact-tool/build_fingers.py \
  outputs/compact-gripper-sourcing-002/pgea-O-S.STEP \
  outputs/compact-fingers-NEW

docker compose run --rm experiment design/compact-tool/review_isaac.py \
  --cad /workspace/outputs/compact-fingers-NEW \
  --output /workspace/outputs/compact-fingers-render-NEW
```

Choose fresh output directories. Stop the other renderer before launching this
one. Desktop GUI mode additionally needs `--gui`, the local X11 socket and
authorization, host display-sharing configuration, and `/dev/dri` device access
on this workstation. The earlier black-window failure rendered offscreen but
did not display the application; passing the display devices resolved it.
Do not treat a startup log as proof that the desktop view is visible.

## Next gates

1. Choose finger/pad materials and edge radii; calculate compliance and mounting
   tolerances. Qualify locating-pin retention and actual fasteners.
2. Add the fixed vacuum nozzle and robot adapter. Check fixture access, hose
   routing and all tool/PCB/camera clearances.
3. Repeat the camera visibility study with this complete tool, including both
   cable leading edge and connector entrance rims.
4. Add actuator and cable-contact models with explicit uncertainty, then test
   retention, slip and fault responses before sensor-driven Pi approach motion.

The existing ROS perception pipeline and its simulator-truth boundary are
unchanged. No compact-tool task controller or hardware command is enabled.
