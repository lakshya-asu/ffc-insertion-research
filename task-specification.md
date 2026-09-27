**Desk-to-PCB FFC assembly — working task contract v0.1**

2026-09-26. FR3 selected. The earlier budget ceiling has been set aside by the user. The final connector and cable remain undecided. This specification distinguishes project requirements from provisional benchmark choices.

**Project outcome.** Starting from a cable on the desk, acquire it, orient the mating end correctly, align it with the PCB connector, fully insert it, secure the connection according to the connector mechanism, release the tool, and verify the resulting assembly. A pre-grasped insertion demonstration is an intermediate milestone only.

**Initial benchmark assumptions.** One cable is separated from other objects; both ends are free. A rigid fixture holds the PCB. The cable can start face up or down at randomized desk locations and in-plane angles, with bounded curl but no knots or overlapping piles. A known connector/cable pair and controlled illumination support the first implementation. These assumptions must be relaxed explicitly if the intended use includes an attached opposite end, clutter, or an unfixtured PCB.

The prototype reference geometry is a horizontal 15-position, 0.5 mm pitch, 0.3 mm cable-thickness ZIF connector, represented by the Hirose FH12-15S-0.5SH(55) external envelope. This is a benchmark candidate, not a purchase decision. The earlier M0 scene had no functional slot; the Isaac workcell now has an explicit aperture using assumed interior dimensions. Slot dimensions, insertion depth, stiffener geometry, allowable forces and latch trajectory remain drawing/measurement inputs. [Vendor reference](https://www.hirose.com/product/p/CL0586-0523-6-55).

**Frames and units.** Controller interfaces use meters, radians, seconds and newtons. USD angular properties use degrees; diagnostic unit-conversion runs label their mass units explicitly. World W has its origin at the arm base on the desk, z up. Connector C is at the slot-mouth reference: +x points into the connector, +y runs across the cable width, +z is the cable surface normal. The tool proxy has +z pointing along insertion, +x across cable width and +y along the cable normal. Its desired world rotation for the horizontal fixture is explicitly stored in code. The cable-tip frame T shares C's axes when correctly aligned. Keep physical contact-side polarity as a separate label because a rectangular geometric tip can appear aligned while flipped.

Represent local error through T_C_tip = inverse(T_W_C) × T_W_tip. Estimate this relative state from local views, while global vision chooses the cable and pickup region. Keep pose uncertainty, visibility, free cable length, curvature and grasp slip separate from arm pose. Do not infer insertion progress from flange travel alone.

**Skill contracts.**

| Skill | Preconditions | Completion evidence | Failure/recovery |
|---|---|---|---|
| Localize | Frames calibrated; fresh global image | Cable end, insulated pickup patch, connector region and confidence | Acquire another view; abort if localization remains ambiguous |
| Prepare latch | Connector identified; tool free | Mechanism in its required insertion state, visually verified | Retract; retry with pose correction; abort excessive load |
| Lift | Accessible pickup patch, acceptable approach clearance | Vacuum signal plus visual cable separation from desk | Release, relocate cup, retry; count failure |
| Secure grasp | Cable held by suction; lower jaw has access | Jaw-force signal, stable tip observation and no measured slip after handoff | Keep suction engaged; reopen or regrasp |
| Condition | Stable grasp | Correct contact face, usable free length, acceptable twist/curvature | Reorient, feed, or place into an auxiliary regrasp fixture |
| Align | Connector ready; tip visible | Relative tip state inside an experimentally measured capture region | Reacquire view; correct alignment; do not push blindly |
| Insert | Capture-state and force-monitoring checks pass | Seating evidence independent of commanded motion | Retract after edge collision, buckling, stall or force limit; bounded retries |
| Secure connection | Cable seated and supported | Latch closed/lock engaged, verified without pulling cable out | Maintain support; correct closure or withdraw and restart |
| Verify and release | Secured connection | Seating, lock and electrical criteria pass after tool release | Count failure; retain evidence and classify cause |

Timeouts and retry limits must be fixed before a controller comparison. Limits on speed, clamp force, insertion force, travel and stiffness come from selected hardware and bench tests. An unconfigured limit is a reason to disable a physical operation, not to invent a permissive default.

**Success and data.** End-to-end success requires seating and locking appropriate to the connector, electrical tests across the required circuits, and no visible damage. Preserve each criterion separately. For an electrically inaccessible target, report mechanical-only verification explicitly. A dedicated test coupon must expose the PCB contacts and provide a way to test the far cable end without assuming an unprovided return path.

Every episode records configuration/version, seed, cable and connector IDs, connector cycle count, start-state description, raw synchronized observations, control targets and measured state, transition events, retries, stop reason, and each verification result. Include timeouts, operator intervention and failed acquisitions in the denominator. Report first-attempt and bounded-retry completion separately, with trial counts and uncertainty intervals.

**Development milestones.**

| Milestone | Deliverable | Exit evidence |
|---|---|---|
| M0: geometric feasibility | Reproducible FR3 scene, proposed tool envelope, endpoint checks | Reach/collision findings with assumptions; no insertion claim |
| M1: mechanical specification | Actual connector/cable drawings, suction-to-pinch CAD, latch access, fixture and camera layout | Handoff can occur without tool/cable interference; forces and travel sized from measurements |
| M2: bench metrology | Pickup, clamp-slip, bend/twist and insertion-force measurements | Identified parameter ranges and held-out mechanical tests |
| M3: deterministic skills | Instrumented pickup/handoff and pre-grasped alignment/insertion | Independent verification and complete failure logging |
| M4: full deterministic task | Acquisition through verified connection with bounded recovery | End-to-end evaluation across declared initial conditions |
| M5: learned improvement | One controlled learning comparison at a time | Gains under equal hardware, observations, data budget and reset distributions |

MuJoCo is retained for M0. Isaac Sim now implements the M1 workcell and M2 numerical checks with segmented and native-shell models; physical material calibration remains outstanding. Select the contact model by its ability to reproduce measured ribbon shape and insertion-force behavior, with runtime reported separately. Do not spend the initial effort training policies against unvalidated flexible-cable contact.

**Immediate next engineering experiment.** Resolve the suction-to-pinch transfer using a single cable and a benchtop jig: the cup must lift the cable far enough for the lower jaw, keep support during closure, and retract without dragging the tip. This is the first end-effector feasibility test. It determines linkage geometry before feed rollers, tactile pads or a latch pusher are integrated. The Isaac simulation now has independent, force-limited deployment and clamp joints. Its suction-to-pinch handoff is under physics testing; this does not yet validate manufactured hardware.
