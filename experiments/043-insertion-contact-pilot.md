# 043 — Side-entry contact commissioning

This milestone builds and executes an insertion-feed bench in Isaac, using the existing full-profile flexible cable and compact PGEA candidate with printed fingertips. It is **not yet the Raspberry Pi insertion skill**. The bench starts aligned with an assumed channel. There is no FR3 approach, camera servo, ROS motion, latch or seating verification.

## Model and control

The cable is free: 100 rigid sections joined by the existing equivalent bending model, with no grasp attachment. The current profile retains assumed thickness, friction, equivalent stiffness and damping. Static bending evidence does not qualify this contact model. Physics uses 0.125 ms steps and the inherited solver settings.

The test channel has a 0.6 mm gap, 12 mm inner width, 4 mm length and a mouth 1 mm ahead of the initial leading edge. These are exploratory fixture dimensions, not manufacturer socket dimensions. Four rigid, fixed kinematic pieces represent its faces; a fifth closes the mouth in the blocked case. Kinematic CCD is unsupported and ignored by PhysX; the final runner explicitly disables that flag. Moving cable bodies retain their original collision settings.

After bilateral pad loads stabilize, a constrained feed axis advances toward a 4 mm travel target at 1 mm/s. The proposed interface runs at 200 Hz. Actor inputs are pad-force magnitudes, measured tool displacement as a feed-encoder equivalent and an ideal fixture-load proxy. The latter sums contact-impulse magnitudes over instrumented fixture surfaces, averaged over 10 ms. It is not a validated six-axis load cell or tactile sensor. Contact counterpart identities and cable body poses do not enter the controller.

The controller retracts up to 0.5 mm after fixture load exceeds an exploratory 0.25 N setting or feed tracking stalls. Missing pad feedback stops the feed at its measured position. Persistent high load stops a retract. These settings are commissioning limits, not known damage thresholds. Peak impulses and material strain still need separate qualification. Reaching the travel target yields `travel_complete_unverified`, never `seated`.

## Layout correction and failed evidence

The first runner inherited the inline tool orientation from the lift bench. Alternative replay views showed that it points the free tip back toward the housing. Per-mesh AABBs against channel/support flagged possible overlaps. AABB overlap does not prove exact solid intersection, but this arrangement is rejected for the approach; the pad-only collision experiment cannot qualify the assembly.

The next revision rotates the housing beside the cable and sets the pad center 12 mm behind the leading edge, following the orientation principle in experiment 035. Its footprint is 3 mm across and 6 mm along the ribbon, on the insulated body. The CAD, inertia offset and pad collider orientation are changed together. Collision checking still includes only the pads on the tool; a separate sampled CAD/fixture bounds audit is required and is not a swept FR3 clearance test.

`insertion-open-001` was interrupted during closure while narrowing unnecessary cable/support contact-report traffic. It is incomplete. `insertion-side-dropout-001` crashed before its first recorded frame with `malloc(): invalid size (unsorted)` while other workers were active. Its cause is unestablished; no task result is inferred. Local logs and source snapshots are retained. A later retry uses a fresh run directory.

## Evidence and reproducibility

The library preserves actual physics videos, recorded body poses, actor observations/actions, offline geometry scores, source snapshots and native physics-disabled USDZ replays. The independent scorer transforms all four leading-edge corners, checking penetration and cross-section fit in the assumed channel. It has no return path to the controller. Initial pose comes from the bench setup, not perception.

Frozen rerun bundles include the script/helper snapshots, cable profile, CAD and a runtime build recipe. They have not had independent physics repeats or fresh image builds. The CAD is hashed at packaging; this pilot did not hash it at acquisition, so historic asset identity is not independently proved. USD playback and rendering are not physics reruns.

The first three inline cases produced entry in the open channel, a 0.5 mm retract at the blocked mouth, and a stopped feed after signal loss. Their housing arrangement remains rejected. Side-grip results are recorded separately in the public suite; do not pool different geometry revisions into a reliability rate.

## Remaining work

1. Preserve the completed side-grip commissioning cases; broaden independent evaluation.
2. Run timestep, contact-offset, friction, stiffness, initial skew and grasp-position sensitivity tests. Audit deformation and raw load transients.
3. Replace the exploratory slot with source-backed Pi socket contact geometry and verify open-latch clearances. Add compliant contacts/latch resistance with declared assumptions.
4. Include the entire tool and environment in collision checks, then establish a clear FR3 approach corridor.
5. Connect independently evaluated camera alignment to bounded motions and these guard interfaces. Add visual buckle/slip evidence and independent seating checks before calling insertion complete.

## Completed side-grip trials

| Case | Controller result | Independent evidence |
|---|---|---|
| insertion-side-open-001 | 4 mm feed completed; seating unverified | Leading-edge corners 3.067–3.069 mm inside the assumed channel; cross-section fits |
| insertion-side-blocked-001 | Fixture-load cap triggered a 0.5 mm retract | Peak 10 ms load proxy 0.291 N; leading edge outside after retreat |
| insertion-side-dropout-002 | Stopped at 0.500 mm feed after injected pad-feedback loss | No measured additional feed over the 0.25 s observation window |

All three side-grip archives pass the sampled per-mesh CAD bounds check against channel/support, with a minimum separation lower bound of 0.750 mm. This excludes continuous swept motion, the robot, hoses and cable/tool interference; it does not replace full collision-enabled testing. The original inline archives retain eight or more possible-overlap pairs.

The final dropout retry explicitly disables kinematic CCD, which PhysX had ignored in earlier runs. The feed policy is unchanged. These are three commissioning cases with a runner revision noted, not a frozen reliability benchmark. The open positive case has zero fixture contact load: it demonstrates geometric entry through an open channel, not electrical contact or seating resistance.

Validation: 183 CPU tests pass, including five new feed-controller tests. Replay exports check recorded tool and cable translations and disable physics. The public report is at [insertion trials](../docs/library/insertion.html); the rejected [inline trials](../docs/library/insertion-inline.html) remain available.

The side-grip positive USDZ was reopened in Isaac and rendered at its start/end states. Two alternative views show the cable and channel; a third becomes completely uninformative at the final pose. Earlier replay-review attempts aborted on that low-contrast view. The revised review retains every image and records pixel contrast instead of mistaking a flat view for proof that playback failed. These are human-review camera positions, not qualified sensor mounts.
