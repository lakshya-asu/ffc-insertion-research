# Compact gripper: jaw geometry review

The supplier geometry supports a compact thickness-pinch concept. It does not yet
qualify the fingers, their mounting, or a cable grasp. This review checks the bare
PGEA jaw mechanism before attaching those parts.

## What we checked

The public [supplier download portal](https://techshare.co.jp/faq/dhrobotics/dh-download-portal.html)
provides both O-B and O-S drawings and STEP assemblies. O-B shows the 94 mm
bottom/rear cable-exit version; O-S shows the 89 mm side-exit version. The
[English catalogue](https://en.dh-robotics.com/wp-content/uploads/2025/06/DH_PGEAPGIA-catalog_V255.pdf)
assigns suffixes differently. We can select a checked geometry for simulation,
but cannot release an ordering code on that evidence.

The O-S assembly contains 21 components. Its CAD is shared by the 2-10 and 15-10
models; geometry alone does not establish the force variant. We retain the
PGEA-2-10 candidate specification separately.

| Measurement | Result |
|---|---|
| Imported inner jaw opening | 11.4 mm |
| Drawing opening range | 1–11 mm |
| Rebase to nominal maximum | Move each jaw inward 0.2 mm |
| Closing axis in supplier coordinates | Z |
| Front attachment plane | Y = −248.2103195 mm |
| Sampled nominal openings | 23, including 1.14 and 1.30 mm |
| Jaw-to-jaw intersection volume | 0 mm³ at every sampled pose |
| Each jaw group against fixed housing | 0 mm³ at every sampled pose |

We translate the lower jaw and its four screws together, and likewise the upper
jaw and its four screws. We do not scale the supplier model. The check uses solid
intersections rather than bounding boxes: these interdigitated jaw bases have
overlapping bounding boxes even when their actual solids are clear.

The sampled sweep is not a continuous clearance proof. It also excludes new
fingers, nozzle, adapter, cable, board and camera. Results are in
[geometry-audit.json](geometry-audit.json); the reproducible, source-hash-guarded
script is [audit_geometry.py](audit_geometry.py). Original runs are preserved in
`outputs/compact-gripper-audit-001` and `outputs/compact-gripper-audit-002`.

## Getting down to cable thickness

The stock 1 mm minimum opening cannot pinch the currently assumed 0.3 mm header
or 0.14 mm cable body. A proposed 0.5 mm inward offset on each fingertip would
give a geometric pad opening of 0–10 mm:

`pad gap = nominal jaw-base gap − 2 × fingertip inward offset`

That places first contact with the assumed header at a 1.30 mm base opening,
and with the assumed body at 1.14 mm. Both bare-mechanism poses passed the solid
intersection check. These are dimensional calculations, not force commands.
Pad compression, tolerances, slip, pressure distribution and cable damage remain
unqualified. We will not use position completion as evidence of a stable grasp.

## Next acceptance gates

1. Build fingers against the supplier mounting drawing, with explicit screw
   engagement, locating features, material and contact-pad dimensions.
2. Audit the assembled tool at the fixture and PCB, including lower-finger
   access, vacuum-nozzle clearance and the required exposed cable end.
3. Render the actual assembly in Isaac and repeat entrance/leading-edge
   visibility checks with the selected macro camera.
4. Add bounded actuator and contact models, then test retention, slip and
   stop/retract behavior before enabling the Pi task's approach motion.

The existing live preview still uses the earlier tool. No compact-tool insertion
or new physics result is claimed by this CAD review.
