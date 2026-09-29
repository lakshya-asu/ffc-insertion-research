# Commercial suction-and-pinch options

Research pass: 2026-09-28 (lab local date). This is a sourcing and mechanism review, not a qualified hardware selection. No purchase or supplier contact was made. Public sources confirm product families; stock, pricing, standalone availability and delivery dates remain unverified.

## Finding

Commercial tools combine suction and fingers. The reviewed sources do not establish a drop-in tool for lifting our flat camera cable from a desk, handing it to opposing pads clear of its contacts, and inserting it into the Pi Zero socket. The best next candidate remains a small commercial parallel gripper with a commercial miniature suction cup and custom tooling. The handoff mechanism still needs design and simulation.

| Candidate | What the primary source establishes | Assessment for this project |
| --- | --- | --- |
| [Zimmer Multi-item gripper](https://www.zimmer-group.com/en-us/products/innovations) | Vacuum and parallel gripper combined; interchangeable cups; guided vacuum assembly; custom fingers offered. | Closest commercial architecture to investigate. Published page does not establish minimum force, nose dimensions, same-object transfer geometry or compatibility with our thin cable. Requesting a dimensional proposal would be necessary before selection. |
| [RightHand Robotics RightPick](https://righthandrobotics.com/item-qualification) | Suction first singulates an item, then integrated fingers stabilize it. [Product page](https://righthandrobotics.com/products) describes three compliant fingers and suction. | Confirms commercial suction-to-finger handling. A warehouse picking system; no reviewed evidence of a standalone precision FFC tool or connector access. Not our preferred starting point. |
| [DH-Robotics PGEA-2-10](https://en.dh-robotics.com/product/pgea) plus miniature vacuum pickup | Replaceable fingertips, 10 mm stroke, 0.8–2 N per jaw, 150 g, 24 V, RS485 Modbus RTU / digital I/O. | Preferred modification candidate because its supplier geometry is already modeled. Vacuum, handoff and compliant fingertips must be added. Force floor remains a qualification issue. |
| [SCHUNK EGP 25-N-S-B](https://schunk.com/us/en/gripping-systems/parallel-gripper/egp/egp-25-n-s-b/p/000000000000310902) plus miniature vacuum pickup | Commercial parallel gripper; this exact variant lists 12 N minimum gripping force. | Mechanically adaptable, but the published minimum force makes it less attractive for this task than PGEA. Vendor force definitions must be compared carefully: this is not a per-jaw equivalence claim. |

The [2023 retractable-cup research gripper](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2023.1066516/full) demonstrates transitions between finger and vacuum grasps. Its 75 mm diameter and 1.3 kg mass, and its status as a research prototype, make it a mechanism reference rather than our purchase recommendation.

## A concrete vacuum component

[Schmalz FSG 5 SI-55 M5-AG, part 10.01.06.00665](https://www.schmalz.com/en-us/products/automation-743270/vacuum-grippers-746238/vacuum-suction-cups-301609/bellows-suction-cups-round-302817/bellows-suction-cups-fsg-25-folds-303001/10.01.06.00665) is a useful initial CAD candidate. The manufacturer lists nominal diameter 5 mm, compressed outer diameter 7 mm, M5 connection, 19 mm height, 2 mm bellows stroke and 1.5 g mass. The listed 0.32 N suction force at −600 mbar is theoretical on a smooth, dry, even surface with no safety factor; it is not a measured cable holding force. The separate 0.8 N pull-off entry must not be substituted for it.

The 7 mm compressed envelope fits within an 11.5 mm cable width only with adequate lateral placement margin. That does not establish sealing on the stiffener or body. Bellows compliance also allows motion, so use this candidate for acquisition rather than assuming it defines the insertion pose. Silicone is not automatically an ESD-qualified or residue-qualified material. Cup material, seal leakage, curl and surface marking require qualification. The 2 mm bellows stroke is passive compliance, not an actively commanded retraction axis.

The proposed vacuum subsystem also needs a controllable source, vacuum/release valves, pressure sensing near the cup and routed tubing. These components are not selected in this pass. A pressure reading alone cannot establish successful retention or safe grasp position.

## Handoff designs to compare

**Integrated pickup and pinch:** put the miniature cup on an independently retractable or translating carrier. It must reach the desk while the lower finger clears the desk. After lifting the cable, its relative motion must bring an insulated grip region into the open jaws. Verify camera visibility, close with limited force, confirm retention, release vacuum and retract the cup before approaching the connector. Merely bolting a cup beside the gripper does not provide that relative motion.

**Pickup and passive presentation fixture:** use a fixed offset cup to place the cable into a fixture with a supported body and accessible end. Release suction, reposition the wrist, then pinch through a relieved opening. This reduces wrist mechanism complexity but adds a placement/regrasp and fixture dependence. It should be evaluated alongside the integrated mechanism, as previously requested.

Use printed parts for mount and routing prototypes. Thin fingers, alignment surfaces and compliant contact pads need separate material and stiffness decisions; do not assume a printed blade has adequate stiffness or a suitable contact surface.

## Contact exclusion is still unresolved

The current task configuration assumes 4 mm exposed contacts and a 6 mm stiffener, measured back from the leading edge. With a provisional 1 mm exclusion margin, only 1 mm remains within the stiffener behind the exposed region. Our 3 mm longitudinal pad footprint cannot fit there. These lengths and the margin are assumptions, not specimen measurements or a proven damage limit.

Both opposing pad footprints must avoid exposed contacts. A blue stiffener on the upper face does not protect contacts on the lower face from the lower finger. Moving the grip farther back onto insulated body avoids the nominal contact region, but increases unsupported length and buckling risk. The previous uniform rigid coupon test establishes neither safe contact location nor compliant-cable insertion.

## Next qualification gates

1. Explicitly model contact strips, stiffener and insulated body on both faces; sweep placement uncertainty against pad and cup footprints.
2. Compare acquisition/handoff sweeps for integrated motion and the presentation fixture, including desk, finger, cup and camera clearance.
3. Test vacuum seal, lift, leak and loss detection using pressure plus camera evidence. Do not attach the cable to the tool by an unconditional joint.
4. Test compliant cable pinching, slip and damage proxies across the actual candidate force range. The previous approximately 0.12 N ideal-drive bench does not qualify the PGEA's advertised 0.8 N minimum per jaw.
5. Re-run held-end visibility and insertion buckling studies with the chosen grasp setback. Qualify the actual communication/feedback rates before connecting runtime motion.

Decision: retain PGEA-2-10 as the provisional pinch actuator, add the Schmalz cup as a CAD comparison candidate, and keep Zimmer as the integrated commercial alternative requiring missing specifications. Do not call any of these a proven FFC insertion solution yet.
