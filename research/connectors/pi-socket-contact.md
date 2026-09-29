# Pi socket entrance and contact geometry

Research 044 · 29 September 2026

The reference drawings show tapered entry surfaces. The housing, slider and metal contact noses must be separate model components: they touch different parts of the cable, and the contacts need spring compliance. A generous funnel would overstate the robot's capture range.

## Zero 2 W

The [official schematic](https://datasheets.raspberrypi.com/rpizero2/raspberry-pi-zero-2-w-reduced-schematics.pdf) identifies J12 as **M00-718-001-022**. [Arducam's mapping](https://docs.arducam.com/Raspberry-Pi-Camera/raspberry-pi-camera-pinout/) identifies a Molex reference for Zero boards, but exact production-part equivalence remains unestablished.

The [Molex drawing](https://www.molex.com/pdm_docs/sd/545482272_sd.pdf), PDF pages 5–6, gives a 54548-2272 reference. Section J–J shows open/locked actuator positions and bottom contact. Insertion depth is 2.5 mm; the separate length-to-contact-point dimension is 1.35 mm. Preserve the drawing datums rather than interpreting either relative to an arbitrary scene mouth.

The 22-circuit mating width is 11.5 ±0.07 mm, thickness 0.30 ±0.03 mm. The FPC recommendation includes R0.3 corners; the separate FFC outline does not specify that rounding. Do not silently round our selected camera cable. Entrance slope and open throat height are not dimensioned in the inspected section. Front-view dimension D is not a certified minimum free throat.

## Pi 4

[TE 1-1734248-5](https://www.te.com/en/product-1-1734248-5.html) is the vertical board-side reference identified by Arducam; Pi4 production BOM equivalence remains unconfirmed. The leading `1-` matters.

Drawing C-1734248 E1 shows entrance bevels and section X–X. The mating FPC is nominally 16 mm wide and 0.30 ±0.05 mm thick, with **C0.3 on the cable corners**. That callout does not specify a housing chamfer. Actuator operation and locked-cable retention force specifications are not cable insertion-force targets.

## Implementation decisions

1. Build the Zero reference first in a local connector frame: fixed housing, open slider, individual contacts, seating stop. Register it into the board assembly after checking orientation and envelope.
2. Preserve the cavity with separate collision pieces. A single convex hull would fill the slot. Check collision offsets against the submillimetre clearance.
3. Read the sourced dimensions in `config/connectors/pi-socket-evidence-v1.json`. Treat undimensioned lead-in length, angle and throat clearance as explicit parameters. Compare square-edge and tapered variants rather than tuning a funnel solely to make insertion succeed.
4. Exercise the existing bounded feed/retract controller with centred, laterally offset, height-offset and skewed entry, then blocked entry. Actor inputs stay sensor-only; buckling, slip and penetration are offline checks.
5. Port the interface to the TE vertical reference with its own dimensions and frame. Latch closure remains a separate skill.

At the close of research 044, the runner still used the generic experiment 043 channel. Experiment 045 subsequently added a supplier-family socket option; see `experiments/045-socket-contact-reference.md` for trials and remaining assumptions. Contact stiffness, friction and damage thresholds are not determined by the customer drawing.

## Sources and reproducibility

`sources-044.json` records manufacturer links, successful mirror URLs and file hashes. Inspected PDFs remain local under `third_party/connectors/research-044/`; vendor PDFs are not republished. TE direct download returned 403 and Molex direct download timed out. Manufacturer-authored drawings were inspected through the recorded mirrors; failed retrievals remain in `downloads.json`.
