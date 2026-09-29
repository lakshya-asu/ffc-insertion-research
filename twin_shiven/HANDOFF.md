# Hand-off to Lakshya, 27 September 2026

What is in this folder, what we found, and the three things we need from your side. Nothing here has been committed to your repository; take what is useful.

## What we built

A simulation twin of the last 10 mm of the insertion: the Pi camera cable's stiffened end held by a six-axis tool, a hinged-chain tail behind the jaws, and the Pi 4 camera socket built from the TE drawing we believe is the part (1-1734248-5). It runs in MuJoCo 3.12.0 on a laptop and emits the same scene as USD for Isaac Lab from one description file.

On top of it: a sensor-only environment whose sensors are models built from datasheets and published measurements (a macro-camera estimate with frame timing, latency, blur, defocus, outliers and occlusion; an ATI Nano17 with its resolution, bias, drift and filter; the FR3's repeatability, settling and delay; an AnySkin pad and a jaw load cell), a "seated" rule decided from those readings and graded against truth, a hand-written expert, and a scoring sweep that reports the probability of a verified seat under a force limit per starting error, with Wilson intervals. `ASSUMPTIONS.md` lists every number and modelling choice with its source class.

Your rules are followed: no simulator truth in the controller, its critic, its reward or its stage transitions; truth is read only afterwards to grade. Your channel names, your frame convention (connector frame at the mouth, +x into the slot, +y across the cable, +z the surface normal) and your seating rule (all four tip corners inside and past the seat) are reused.

## Files

- `scene/ffc_twin.usda`: the twin as USD. Tool is an articulation with six drives, the header a rigid body, the tail an articulated chain, TGS at 120 Hz with 192 iterations (the Factory task settings). PhysX attributes are authored by name because plain USD lacks your PhysX schema plugin; Isaac resolves them on load.
- `scene/ffc_twin.xml`: the same scene for MuJoCo.
- `scene/ffc_twin_spec.json`: every number with its source. The `placeholders` list is what measurements must replace.
- `results/region_expert_good_L1_n200.svg` and `.json`: the score grid, expert with a good estimate (off by 0.05 mm, 0.3 degrees). 200 runs per cell.
- `results/region_expert_poor_L1_n200.*`: the same with a poor estimate (0.15 mm, 1.0 degree).
- `results/region_push_none_L1_n50.*`: a straight push with no correction.
- `results/detector_grade.json`: how often the seated rule was wrong, both ways, against truth.
- `drawings/`: the connector and cable drawings and the board schematics we found.
- `code/`: the twin, environment, sensor models, seated rule, expert, tests and scripts. `README.md` has the run commands; `ASSUMPTIONS.md` is the register.
- `guide/how-we-solve-it.html`: the stage-by-stage explanation with the simulation clips inlined. Opens in any browser. `plan.html` is the working plan with the decisions and the connector research.

## What we found

| Controller | Sensing | Seated under 3 N, mean over the grid |
|---|---|---|
| Expert | lit, focused macro cameras, matched pad | 94 percent |
| Expert | low light (4x pixel noise), mismatched pad | 95 percent |
| Straight push, no correction | none | 0 percent |

The grid is sideways error from -1.5 to +1.5 mm crossed with turn error from -6 to +6 degrees, with height, pitch and roll randomized up to 1.5 mm and 6 degrees, and stiffness, friction and tool gains randomized each run (20 runs per cell in this pass; the 200-run pass is next). With a macro camera at 91 px/mm, four times the pixel noise is still a few microns, so the estimate is not what limits the score; the remaining failures come from contact and grip.

Seated-rule grade (60 expert runs, 40 deliberate partials, 20 jams): 1 false "seated" in 61 negatives and 0 misses in 59 positives with the good sensing; 0 in 61 and 0 in 59 with the poor.

Four findings from modelling things properly rather than assuming them:

1. The rigid end of the cable is the 6.3 mm support tape, not 11 mm (the contacts sit on the tape's other face). With 3.45 mm inside the slot only 2.85 mm of tape is outside, so the jaws must be about 1.5 mm long, and whether they clear the lifted latch is an open question on the TE drawing. Gripping the thin film behind the tape instead lets the tip pitch by tenths of a millimetre under sub-newton loads and fails.
2. The grip must be modelled as dry friction, not a weld. Your 0.36 N simulated clamp holds 0.29 N of axial load; jams reach 1 to 2 N. About 3 N of clamp is required for a 1.5 N force ceiling with margin. A naive push into a jam slides the cable out of the jaws.
3. Gravity for the Pi 4 points into the slot (upright socket); the twin now sets it per board.
4. A camera estimate must be timestamped against the tool pose of the same frame, and tool travel during flagged slip must be discounted, or the dead-reckoned depth is wrong once the mouth is hidden.

## What is still a guess

The open-slot height (0.5 mm assumed; the drawing does not give it), the size of the side chamfers (drawn, undimensioned), the laminate modulus (3 GPa), pad and housing friction (0.2 to 0.6), the jaw-to-latch clearance, the 3 N force limit, and the camera exposure and processing times. `ASSUMPTIONS.md` has the full register: 20 datasheet numbers, 13 published, 23 derived, 14 placeholders, 31 modelling choices with their consequence.

## Three asks

1. Isaac Lab 3.0.0-EA beside your Isaac Sim 6.1 container. It pairs with 6.1; the release notes are at github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0-EA. Then load `scene/ffc_twin.usda` as an articulation and check that a straight push seats the header at the same depth and force class; your `seating_metrics` should grade it unchanged.
2. A photo of the Pi 4 camera socket housing under magnification (TE parts carry an AMP mark and are 22.4 mm long; the common clone is 22.0), and a caliper measurement of the slot height with the latch lifted. That single number is the biggest placeholder.
3. The product specifications the drawings point to, downloaded in a browser since the sites block scripts: TE 108-57080, Molex PS-54548, Amphenol SC-SFW10. They hold insertion force and rated cycles.

## What is next on our side

Generating the expert's training set (20 000 runs with corrections), then the first learned controller, a copy of the expert scored on the same grid. The interesting test is the poor-estimate case: whether a learned controller can beat 25 percent by reacting to force where the expert only retries.
