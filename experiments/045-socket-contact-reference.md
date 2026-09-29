# 045 — Side-entry socket contact reference

29 September 2026. Isaac 6.1.0, local cable/tool bench. No FR3, board registration, camera servo or latch closure.

## What changed

The generic 4 mm channel has a separate supplier-family reference option. It uses the Molex drawing's 2.5 mm insertion depth and 22 contacts at 0.5 mm pitch. It includes a housing, open slider roof, entry guides, seating stop and individual spring-supported contacts. Exact equivalence to the Zero 2 W M00 production part is unestablished.

The 11.65 mm throat width, 0.40 mm gap, 0.30 mm guide length, 0.15 mm expansion per side, contact shape and spring/friction parameters are hypotheses recorded in the frozen per-run configuration. The feed remains sensor-only: pad loads, encoder-equivalent travel and an ideal instrumented-fixture load proxy. Cable and contact poses are offline observations only.

## Initial comparison

| Run | Geometry | Outcome |
|---|---|---|
| socket-centered-001 | Centred, tapered guides, box contacts | Native allocator crash after first frame; incomplete |
| socket-centered-002 | Identical retry | Load-triggered retract; leading edge reached only about 0.955 mm depth |
| socket-offset-001 | Same geometry, 0.15 mm lateral offset | Feed complete; final leading-edge corners 2.345–2.403 mm deep and inside assumed throat |
| socket-square-offset-001 | Same offset/contact model, square entrance | Load-triggered retract; final leading-edge corners do not fit throat |

The paired offset comparison supports an entrance-guide benefit for this one setup. It does not establish a reliable capture range. The offset run moved the first cable segment about 0.102 mm laterally. No commanded-travel result is called seating.

## Contact geometry corrections

The centred box-contact run stopped near the front face of the contacts: 0.95 mm from the assumed mouth. This suggested the artificial vertical nose was catching the cable. The manufacturer section shows a tapered contact profile.

`socket-tapered-001` replaced the box nose with a convex mesh. PhysX warned that it enlarged the thin mesh; the run stopped before feed with a large fixture load. Its schema defaults to a 1 mm convex minimum thickness. `socket-tapered-002` explicitly reduced that limit to 10 micrometres, but still produced unintended load. An empty-socket diagnostic identified adjacent contact meshes colliding despite their authored gaps.

Revision 4 uses an inclined analytic box plus a flat land for each spring-supported nose. This avoids convex cooking. It does not disable collisions between neighbouring contacts. The contact/floor pair is filtered at the sprung support; the floor is a simplified housing, not a model of the individual moulded beam pockets. The first empty-socket check returned no contact reports. The corrected dynamic run is recorded separately as `socket-tapered-003`.

## Evidence and limits

Original MP4s, physics-disabled USD replays, sensor traces, offline geometry scores and source/config/CAD rerun bundles are kept in the motion library. Replay now records and checks moving contact positions as well as cable/tool motion. The earlier offset replay was reopened and rendered from four views in Isaac; the frontal view is partially occluded by the tool.

Only tool pads have collision shapes. Full tool clearance, board integration, identified contact forces, material damage and seating remain unqualified. The cable is a free articulated approximation with assumed damping. Fresh image builds and independent physics repeats of these new runs have not been tested.

## Corrected revision results

`socket-tapered-003` reached 2.5007–2.5040 mm leading-edge depth with the corners inside the assumed throat. Feed completed and peak 10 ms fixture-load magnitude was 0.1116 N. The 2.5 mm backstop boundary is exceeded by up to 3.995 micrometres. This is an observed entry with a failed strict containment check, not verified seating. Do not increase the acceptance tolerance to relabel this run. Check timestep/contact convergence before qualifying the contact result.

`socket-tapered-blocked-001` triggered the load guard (peak 0.3001 N), retracted the tool by 0.5 mm, and ended with the entire leading edge approximately 0.495–0.500 mm outside the mouth. This does not establish a cable damage limit or recovery reliability.

The revision-4 empty socket produced no contact reports over 200 steps at 0.125 ms. Native replay of the corrected entry was reopened and rendered in Isaac. Scorer revision 4 separates reaching the review depth from strict cavity containment and explicitly reports backstop overrun; revision-3 scores are retained beside the updated scores. CPU checks: 186 tests, including offset-frame and backstop-overrun scorer checks.
