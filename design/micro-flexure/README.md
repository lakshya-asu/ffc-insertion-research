# Compact suction and pinch head

A design-review candidate for the Pi Zero ribbon-cable task. The complete modeled head is **30 × 28 × 24.7 mm**, including the fingers and offset suction pickup. The central body is 22 × 20 × 23 mm. The FR3 adapter, wiring and tubing bend radii are additional.

## How it works

A fixed lower steel blade supports the cable. A moving upper finger presses onto its insulated body, 8.5–10.5 mm behind the leading edge. This leaves the modeled contacts and stiffener exposed for inspection and insertion. Replaceable pads are 2 × 6 mm. Four thin steel foil flexures guide the upper assembly, and a separate steel sensing leaf carries the upper pad. Bonded strain gauges would measure its bending; the electronics and exact gauges remain to be selected.

The actuator candidate is the [H2W NCC01-04-001-1X](https://www.h2wtech.com/product/voice-coil-actuators/NCC01-04-001-1X), a bare moving-coil actuator with an 11.1 mm diameter housing. It needs our guidance mechanism, current driver and position feedback. Its CAD representation uses public envelope dimensions and provisional internal geometry; it is not supplier-certified CAD.

The offset suction tip represents a Schmalz SUF 3-sized pickup. The proposed sequence is suction pickup, placement on a relieved passive fixture, vacuum release, wrist repositioning, then pinch. Fixed suction and fixed jaws cannot hand off a cable merely by moving the wrist. The fixture and complete handoff still need qualification.

The frame and precision spring parts should be metal. Printed parts can serve as covers and suction brackets. Printing the 40 µm steel guide leaves is not proposed. A small stop limits sensing-leaf travel; it does not limit contact force.

## Preliminary engineering checks

The nominal CAD solids are valid. Twelve sampled jaw openings check the moving assembly against selected fixed parts. An independent 51-position AABB screen gives a minimum 5.5 mm tool-to-board/socket separation in the assumed straight approach. This excludes the cable, full robot, adapter, leads and manufacturing tolerances. Sampled sightlines assess tool obstruction only, not optical resolution, board self-occlusion or cable occlusion.

Analytical beam estimates use an assumed steel modulus of 200 GPa and a 3 g moving assembly. An exploratory 0.15 N pad load requires approximately 0.235 N actuator force including guide restoring force and gravity, versus the supplier's 0.27 N continuous rating. Estimated current is 0.52 A and copper dissipation 0.41 W. The approximately 0.035 N remaining margin excludes wire forces, temperature effects and tolerances. The pad-force target is not a verified damage limit.

Estimated guide stress is 207 MPa; sensing-leaf deflection is 0.182 mm under that load. These are beam calculations, **not FEA or fatigue qualification**. Guidance includes a small modeled parasitic sideways shift. Coil clearance and leaf attachment details need tolerance analysis.

## Review artifacts

The rendered views show the actual tessellated CAD. The opening/closing GIF and USD timeline prescribe geometry from a 1.2 mm gap to first touch on a 0.14 mm body coupon. They do not simulate squeeze, strain, suction or insertion. The Pi Zero board is existing community CAD with an assumed open socket for scale review.

Open `closed.step` or `open.step` in a CAD application. Open `micro-flexure.usdz` in Isaac Sim for geometry inspection and timeline playback, frames 0–72 at 24 fps. Physics is deliberately absent. This turn could not access Docker, so the new USD was checked with the local USD library, not launched in Isaac.

Reproduce from the repository root using Python 3.12 with CadQuery 2.6.1, numpy, matplotlib, Pillow and usd-core, plus a C++17 compiler:

```bash
python design/micro-flexure/build.py --output outputs/micro-flexure-new
python design/micro-flexure/render.py --output outputs/micro-flexure-new
python design/micro-flexure/review.py --output outputs/micro-flexure-new/clearance.json
```

Use a fresh output directory. Rendering also needs `third_party/raspberry_pi/zero/zero2w.usdc` from this repository. The build records its own source and parameter hashes; review packages additionally record source and asset hashes.

## Before adopting it

Confirm actuator attachment details and coil travel against supplier CAD. Select position sensing, gauges and driver. Check force range across temperature, wire routing, tolerances and flexure fatigue. Finish the adapter and fixture. Then qualify actuator dynamics, finger compliance, suction retention, cable handoff and camera visibility in simulation before integrating with the existing sensor-driven robot controller. Existing commercial and PQ12 alternatives remain in the shortlist.
