# Desk pickup to insertion: mechanism comparison

Status: engineering hypotheses for discussion, not validated hardware. FR3 is selected; connector and cable are not. Compare fixture-assisted handling and handling entirely within the tool before committing to fabrication. Budget is not used to reject concepts at this stage.

## Shared requirements

Start with one separated cable on the desk, both ends free, a fixed PCB, and controlled lighting. Detect the intended end and exposed-contact face. Acquire it without contacting exposed conductors where practicable, establish a stable insertion grasp, measure the tip relative to the tool, insert without damage, operate any latch, and verify electrical connection. These assumptions must be revisited for tangled cables or a preconnected opposite end.

The existing workcell model tests static reach and collision envelopes only. Its primitive suction slide and jaws do not establish a mechanically possible transfer. Moving the robot preserves the suction-held cable's pose relative to fixed jaws; a relative mechanism or external support is needed.

## A: Integrated vacuum datum and deployable lower jaw

A small compliant suction cup sits within an upper support shoe. The lower jaw initially parks outside the cable's projected width. Suction acquires an insulated or stiffened patch behind the contacts. After lifting, the lower jaw slides sideways underneath the cable at clearance, then moves upward to clamp it against the shoe. Vacuum is vented after the clamp is verified.

This avoids drawing the cable through a fixed lower jaw. The initial prototype should provide independent deployment and clamping motions; coupling them with a cam can follow once the required trajectories are measured. The cup must either compress enough to seat the cable against fixed support pads or retract through a short independent stroke. Passive seating is a hypothesis to test, not an assumed property of a suction cup.

Sequence: observe → suction contact → pressure plus visual acquisition check → lift and check jaw swept volume → deploy jaw → clamp → vent → remeasure tip → orient → approach connector.

Main risks: a curled cable occupies the lower-jaw path; vacuum deforms the tip; clamping shifts its pose; a single cup permits rotation; the tool obscures the tip or collides with the connector. Full-cable lifting and wrist rotation may correct face orientation, but tail sweep and joint limits must be checked. This mechanism does not intrinsically straighten or flip arbitrary cables.

## B: Suction pickup and passive regrasp nest

Use a small vacuum pickup alongside a conventional pinch gripper, or exchange tools if justified. Place the cable's stiffened end on a raised shelf. Provide an open underside and lateral access for the pinch jaws, plus a broad tail support. Release suction, reposition, and pinch the protruding end. An overhead camera measures the actual position after release.

The shelf may include generous lead-ins and an adjustable datum, but narrow channels must not scrape exposed contacts or wedge a curled cable. A passive nest cannot be assumed to retain the cable when tail weight pulls on it. Test support geometry and friction first. If a powered clamp or nest vacuum becomes necessary, classify it as an active fixture and include that extra actuation in the comparison.

Sequence: observe → suction pickup → place on nest → confirm stable support → vent → inspect → regrasp → remeasure tip → approach connector.

Main risks: the cable slides after release, tail weight pulls it out, the gripper cannot approach the shelf, or placement adds more errors than the fixture removes. The nest supports inspection and alternative grasps, but automatic face flipping needs an explicit maneuver and suitable geometry. It is not solved by adding a shelf.

## Comparison to test

| Criterion | Integrated tool | Passive nest |
|---|---|---|
| Cable support during transfer | Continuous vacuum until clamped | Nest must support cable after vacuum release |
| Wrist mechanics | Two jaw motions; potentially cup retraction | Conventional pinch plus vacuum pickup |
| Cell complexity | More compact station | Extra calibrated station and travel |
| Visibility | Jaw and cup occlusion is a design constraint | Dedicated inspection view is easier to arrange |
| Face correction | Wrist roll with tail clearance or additional mechanism | Alternate grasps possible, explicit flip still needed |
| Connector adaptability | Replaceable tips and cup location | Replaceable nest and fingertips |
| Expected cycle time | Potentially fewer movements; unmeasured | More movements; unmeasured |
| First useful experiment | Pickup-to-clamp transfer with tip tracking | Placement-release-regrasp with tip tracking |

Recommendation: retain both. Use the nest as a baseline to isolate insertion from in-tool transfer. Develop the integrated design if its measured reliability and cycle time justify its complexity. Do not select from appearance or claimed precision alone.

## Dimensions and forces that drive the choice

The free cable length beyond the jaw must accommodate insertion depth and tool-to-connector clearance, while remaining short enough to resist buckling and tip pose uncertainty. If these bounds do not overlap, revise fingertip geometry or add controlled feeding. Raising insertion force is not a remedy.

For intuition only, ideal beam buckling scales as EI/L² under fixed boundary conditions. Halving unsupported length increases the ideal critical load fourfold. A laminated cable, its stiffener, curvature, and contact constraints require measurements; this scaling is not an operating force limit.

For symmetric pinch contact, a simple friction estimate is 2μN ≥ S(F_insert + F_tail), where N is each jaw's normal load and S is a margin. Measure friction, actual insertion forces, and damage onset. Motor current is not a direct calibrated contact-force measurement.

Ideal suction force is Δp A: a 3 mm circular effective area at 30 kPa gives approximately 0.21 N before losses. This is an illustrative calculation, not a cup specification. Curvature, leakage, peel moments, desk adhesion, and tail drag can dominate. Vacuum pressure alone cannot distinguish lifting a cable from sealing against the desk.

Latch operation must preserve the insertion grasp until seating is secure. A fixed pusher moved by the same wrist may also move the cable; an independently actuated pusher or external latch station may be needed. Connector selection precedes final tooling dimensions.

## Matched screening experiment

Use the same cable and connector for both concepts. Start on a bench jig before involving FR3. Measure actual cable thickness, stiffener length, contact-face geometry, curl, and connector insertion depth from the chosen parts and drawings.

Vary exposed face, pickup offset, initial yaw, natural curl, and tail support. Begin with exploratory repeats per condition, then size the comparative trial using observed variability. Log pickup failures, transfer failures, tip translation and rotation before/after transfer, successful insertions, damage, time, retries, and electrical verification. Report success per attempted cycle, including recovery cost; do not exclude failed pickups from the denominator.

Use an overhead view for pickup and yaw, and an oblique or side view for height, pitch, and buckling. Re-estimate the tip after venting or regrasping. Success means a correctly seated, undamaged, electrically verified connection, not merely a plausible image.

## Research anchors

- IROS 2022, *On Robotic Manipulation of Flexible Flat Cables Employing a Multi-Modal Gripper with Dexterous Tips, Active Nails, and a Reconfigurable Suction Cup Module*, DOI: https://doi.org/10.1109/IROS47612.2022.9981313. Prior art motivating combined acquisition and constrained manipulation; it does not validate the mechanisms proposed here.
- Hirose FH12 example part and associated drawings: https://www.hirose.com/product/p/CL0586-0523-6-55. Example for connector-specific geometry, not a frozen component choice.
- Existing local hardware survey: hardware-selection.md.
