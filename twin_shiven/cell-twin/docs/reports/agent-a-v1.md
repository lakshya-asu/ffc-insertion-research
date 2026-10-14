# Agent A report, training build v1

Branch `feat/training-v1-a` (worktree `~/Professional/cell-twin-wt-a`). Python: `~/Professional/cell-twin/.venv/bin/python` (the venv is not in the worktree). All runs were made with the laptop shared with agent B (load average about 10 at the throughput run), so wall-clock numbers are lower bounds on speed.

## CHANGES

- `cell_twin/training_config.py` (new, committed first): the spec's shared interface (TICK_HZ, WINDOW, ACTION_BOUNDS, OBS_LAYOUT, OBS_DIM, ACTION_DIM, CONTACT_CLASSES), plus `ACTION_SCALE`, `CURRICULUM`, `OUTCOMES`, `measured_torque_noise()` and the `Randomisation` dataclass with `sample(rng, stage)`, `to_dict()` and `from_dict()`. Tilt, slip and female tilt are sampled as magnitudes in the spec ranges with a random sign, and recorded signed.
- `scripts/torque_noise.py` (new, A1): static holds (every joint under 0.002 rad per tick for at least 15 ticks) in the 23 episodes; per-joint std after removing a straight-line trend in each window, pooled by window length, written to `out/torque_noise.json` with the raw std, the tick-to-tick std and the window counts.
- `cell_twin/bench.py`:
  - A1: `TORQUE_NOISE_NM` now comes from `out/torque_noise.json` (right arm, per joint), 0.05 Nm if the file is absent.
  - A2: `Bench(..., rand=Randomisation)` and `Bench.apply(r)`: right-arm kp and kd scales, part friction on every part geom, the leaf time constant times `contact_stiffness_scale`, latencies; `bench.rand` holds the draw. `pick_errors(r)` turns a draw into the header and housing pick errors (they are compiled into the model, so they go to the constructor).
  - A2: torque sensing is sampled once per 30 Hz tick (`_sense`, called from every physics tick): scale, bias, Gaussian noise, a 1-tick low-pass (0.5) and the observation latency queue; `torques()` has no side effects and returns the same value if read twice in a tick. The action latency queue sits in `_command`; both latencies switch on only after `reset` finishes.
  - `step_action(a)`: one tick of a policy action in ACTION_BOUNDS units, clipped to [-1, 1].
  - Sensor-only fix: `step_tool` pivoted rotations about the TRUE pin tip (`pin_tip_2`); it now uses the nominal tip (what the cell knows).
  - Servo fix: `_servo` now adds the commanded twist as feedforward and corrects the measured pose error at gain 0.05 per tick (`SERVO_GAIN`, `SERVO_ROT_GAIN`). The old error-only loop at 0.45 overshot a 3 mm lateral move by 1.3 mm and a 5 mm push by 1.5 mm; the new loop overshoots by 0.4 and 0.5 mm.
- `cell_twin/controllers.py`:
  - `push_only` and `align_by_contact` take their torque baseline over 5 held ticks (reading 5 times in one tick no longer averages noise).
  - A3: `ContactMonitor` (tau_ext = torque minus a lagged free-motion baseline frozen at onset; onset from a noise-weighted matched filter on joints 1 to 3 at 3 sigma for 2 ticks, or a weak rise while the travel rate halves; a pop-through detector; re-arms after a retract). `Expert` with `act(b)` returning one ACTION_DIM action: approach (slower through the face zone, holding the across-row position), classify the contact by onset travel (surface, entry, header-on-face, nothing), and pick the next across-row shift from a feasibility grid over the offset with a prior favouring small offsets, which gives the coordinator's widening, alternating miss branch; on entry a tilt-to-null of up to 6 steps of 0.5 deg about the pinch axis (bounded at 8 deg) on the wrist moment, push, a force-regulated hold, back off 1 mm and re-tilt on a wedge. `Noisy` / `noisy(expert, sigma)` (DART: executes clean plus N(0, sigma), keeps `.clean`). `expert(b, ...)` runs it and returns executed actions, clean actions and the detector's firing tick.
  - A4: `SeatDetector` / `seat_detector(b)`: step once per tick; fires after 3 consecutive ticks of: travel since onset in 4 to 16 mm, the leaf pop-through seen since onset (3-tick travel advance of at least 1.4 mm while pushing at 0.4 Nm or more), stall (under 0.1 mm over 8 ticks) and push-axis tau_ext at or above 3 Nm.
- `scripts/capture_region.py`: `--v1 grid|stage` runner (multiprocess, push, align or expert, the seat detector stepped beside the controller, graded against `Bench.seated()` at the firing tick and at the end, Wilson intervals, results in `out/<name>.json`). The old mode is unchanged.
- `scripts/throughput.py` (new, A5).

Commits: c04a216 (interface), 5653134 (A1), c59dbda (A2), then the A3/A4 series up to the report commit.

## THINGS I DID NOT TOUCH

- `cell_twin/scene.py`, `app/`, `scripts/register_*`, everything agent B owns.
- The grasp weld (`GRASP_SOLREF`) that lets the header slide about 1.5 mm per Nm in the fingers; it dominates the travel-to-force relation (see CONCERNS).
- Female housing gains (left arm): `apply` randomises the right arm only.

## MEASUREMENTS

Every number below comes from the command next to it, run in the worktree.

1. Smoke test before any change (`Bench(seated, PickError(skew_deg=4, lateral_mm=(0,1.0)))` + `align_by_contact`, main and worktree identical): **F-miss, depth -1.69 mm**, not the expected S near 6 mm. With zero pick error it was also F-miss: the 7.5 mm approach budget runs out before the leaves give the first torque rise (rise starts at about 7.7 mm of travel), so the controller backs out of a clean entry.
2. A1 torque noise (`python scripts/torque_noise.py`), Nm, detrended std, joints 1 to 6:
   - right: 0.039, 0.238, 0.109, 0.053, 0.020, 0.013 (252 windows, 9129 ticks; raw 0.070, 0.503, 0.133, 0.086, 0.031, 0.018; tick-to-tick 0.015, 0.057, 0.042, 0.015, 0.008, 0.006)
   - left: 0.052, 0.029, 0.013, 0.019, 0.014, 0.002 (55 windows, 12204 ticks)
   - effort resolution: 0.0137 Nm (joints 1 to 3), 0.0049 Nm (4 to 6).
3. Contact geometry probe (nominal bench, straight push, no noise; scratch script `probe.py`): the outcome depends on the pin row's across-row tip offset only. |offset| under about 1.0 mm enters and seats even at 8 deg tilt; 1.0 to 1.7 mm: pins on the face, onset at 4.6 to 5.4 mm of travel; 1.7 to 2.7 mm: header on the housing face, onset at about 10.4 mm; over 2.7 mm: no contact. The wrist torques carry no sign of the offset (all about -0.05 Nm), so the search has to try both sides.
4. Regression check requested by the coordinator (20 episodes, tilt uniform 0 to 8 deg, slip 0 to 4 mm, `align_by_contact`, scratch `reg.py`): main a6adc3e **3/20 S**; branch before the baseline fix **1/20**; branch with flat 0.05 Nm noise **0/20**; after the held-baseline fix (c215bef) **1/20**. In my runs every episode ran 900 or more servo ticks; I could not reproduce an 8-tick ending (also checked on an export of c59dbda). align_by_contact cannot cover stage 2 on either branch (episode 16, tilt -0.1 deg slip 0.3 mm, fails on all three).
5. Expert capture grid, nominal bench, n = 20 per cell, slip {0,1,2,3,4} mm by tilt {0,4,8} deg (`python scripts/capture_region.py --v1 grid --controllers expert --n 20 --slips 0,1,2,3,4 --tilts 0,4,8 --procs 2 --out capture_expert_n20.json`): **295/300 = 98.3 percent [96.2, 99.3]**. Every cell 100 percent [83.9, 100] except slip 0 tilt 4: 95 percent [76.4, 99.1], slip 3 tilt 4: 80 percent [58.4, 91.9]. Mean 218 ticks per episode.
6. Expert over the full stage-2 randomisation, n = 400 (`python scripts/capture_region.py --v1 stage --stage 2 --controllers expert --n 400 --procs 2 --out stage2_expert_n400.json`): **257/400 = 64.3 percent [59.4, 68.8]**. Outcomes: S 257, F-miss 122, F-partial 13, F-jam 8, X 0. 107 of the 143 failures hit the 600-tick limit (the search under randomisation is too slow). Ablation at n = 20 (scratch `abl.py`, same seeds): all randomisation 14/20; pick error only (nominal bench) 20/20; earlier ablations of single groups (latency, gains, sensing, contact, female tilt) each moved the count by 1 to 3, no single group explains the drop.
7. A4 seat detector against `Bench.seated()`:
   - stage 2, n = 400 (command in 6): fired while not seated **1/400 = 0.25 percent [0.04, 1.4]**; fired on episodes that ended unseated 1/143 = 0.7 percent [0.12, 3.85]; **FN 204/257 = 79.4 percent [74.0, 83.9]**. By outcome: S 53 of 257 fired, F-miss 1 of 122, F-partial 0 of 13, F-jam 0 of 8.
   - nominal grid, n = 300 (command in 5): FP 0/300 [0, 1.26]; FN 112/295 = 38.0 percent [32.6, 43.6].
8. A5 throughput, 1 process (`python scripts/throughput.py --procs 1 --episodes-per-proc 12`, load average 10.2 from agent B's run): **32.3 episodes per minute, 177.7 ticks per second (5.9x real time)**, model build 0.75 s per episode, 329 ticks per episode on average. 4 and 8 processes not measured (CPU held by agent B, per instruction).
9. Servo step response (scratch `servo.py`): 3 mm lateral at 0.3 mm per tick, 5 mm push at 0.5 mm per tick. Old (error-only, 0.45): peak 4.26 mm and 6.47 mm. New (feedforward + 0.05): 3.39 mm and 5.53 mm.

## Pass criteria

- Expert seat rate at least 90 percent over stage 2 at n >= 400: **FAIL**, 64.3 percent [59.4, 68.8] (98.3 percent on the nominal-bench grid).
- Seat detector FP under 1 percent and FN under 5 percent: FP **PASS** (0.25 percent, upper bound 1.4), FN **FAIL** (79.4 percent).
- Throughput measured: **PARTIAL**, 1 process only.
- A6 capture grid (push_only, align_by_contact, expert, n = 40, slip 0 to 5 by tilt 0 to 8): **PENDING CPU**. The runner is ready: `python scripts/capture_region.py --v1 grid --controllers push,align,expert --n 40 --slips 0,1,2,3,4,5 --tilts 0,2,4,6,8 --procs 8 --out capture_v1.json` (3600 episodes, about 15 to 20 min at 8 processes if each sustains about 30 episodes per minute).

## CONCERNS

- The detector's FN is the travel band and the stall test, not the pop: the grasp weld lets the tool keep travelling about 1.5 mm per Nm after the seat, so "stall" needs the expert to stop pushing, and under randomised gains the held force keeps creeping. A detector built on travel since onset is fragile here; the pop-through signature (a jump in travel at almost constant force) is the cleanest seat cue I found, and it is what keeps FP at 0.25 percent.
- The grasp weld also moves the tip about 3 to 6 mm along the row after face contact in some randomised episodes (traced in episode 3 of the stage draw), invisible to the encoders. That is the part slipping in the fingers; the expert cannot see it and the search then hunts across the row only.
- The measured noise on right joint 2 (0.24 Nm) is probably inflated by contact loads during holds; times 2 in the randomisation it is 0.48 Nm. It weakens torque onset detection; the expert leans on joint 1 and the encoders for that reason.
- The servo change (feedforward) changes the bench for agent B as well; B's recorded data before 0a8c30f used the old servo.
- `contact_stiffness_scale` multiplies the leaf time constant, as the spec table says; a larger value means a softer leaf, the opposite of what the name suggests.
- The expert uses bench priors: the 4 mm approach standoff and the onset travel bands (surface, entry, header-on-face) measured on the nominal bench. They are geometry priors, not truth, but they move if the approach hand-off moves.

## OPEN QUESTIONS FOR SHIVEN

1. The expert replaces the spec's "existing spiral search" with an across-row search ordered by contact class (the spiral would need about 280 probes to reach a 5 mm offset). Accept?
2. The grasp weld lets the header slide in the fingers under load. Is that real on the cell (then the detector must not use travel since onset as a seat cue), or should the weld be stiffened in `scene.py` (not my file)?
3. Torque noise: use the detrended static-hold estimate (0.24 Nm on joint 2, kept) or the left arm's quieter figure as the sensing noise for both arms?
4. Should the stage-2 pass bar be judged at 600 ticks (20 s)? 107 of 143 stage-2 failures were still searching at the limit.
5. Agent B (via the coordinator) reported 8-tick episodes with align_by_contact on c59dbda; I could not reproduce it with Bench + align_by_contact directly. If it persists in B's env it is in the env wiring, not the bench.

# Round 2 (2026-10-08)

Merged main (rigid grasp weld GRASP_SOLREF 0.001, 900-tick limits, agent B's files). Runs were capped at 3 processes after the two kernel panics. Every number comes from the command next to it. The `out/*.json` files are gitignored, so they exist only on this laptop.

## CHANGES (round 2)

- `scripts/capture_region.py` (9576e38): `--weld <s>` sets the grasp weld per run, `--log-ticks` saves per-tick torque, travel, truth seated, depth and expert actions to `out/ticklogs/<out stem>/`, and `--ablate g1,g2` keeps the pick error plus the named randomisation groups (latency, gains, sensing, contact, female, friction, all, none; `a+b` combines them) and reports a summary for each group.
- `cell_twin/controllers.py`:
  - Expert waits 45 ticks after any contact before it shifts (commit e07f6a6). The soft-held housing (left wrist kp 10) moves about 1 to 1.5 mm along the row under a face contact and creeps back with a time constant of about 40 ticks (scratch drift.py: 0.67 mm after 10 ticks, 0.26 mm after 50, 0.10 mm after 90). Without the wait the next probe lands on the face with the across-row offset already correct, and the search then moves the wrong axis.
  - Seat detector v2, which no longer needs the pop (4826616): fires after 3 consecutive ticks of (a) travel since onset minus 1 mm per Nm of push between 2.8 and 12 mm, (b) stall: under 0.1 mm over 8 ticks with no retract commanded in that window, and (c) push-axis tau_ext at or above 2.5 Nm. The compliance term and the 2.8 mm floor come from 320 logged stage-2 episodes: seated stalls gave 3.6 mm or more (1st percentile), leaf wedges 0.3 to 2.0 mm. With the rigid weld the pop-through shows in under 10 percent of seats, so v1 never fired.
  - Expert hold: the tool now stalls. The servo setpoint is brought back to the measured pose, using the expert's own commanded advance minus the measured travel. The force stays because it lives in the integrated joint targets. Before this change the setpoint led the blocked tool and its error term kept creeping the tool in, so the tool never stalled while the push continued.
- `cell_twin/bench.py`: `cmd_z_hist` records the commanded tool-axis advance per tick. These are the controller's own commands, and the detector's stall test reads them.
- Tried and reverted: an along-row dither after a surface contact (the tip moved about 2 mm per 0.9 mm commanded, because the housing moves too), and a roll dither of plus or minus 3 deg after a surface contact. The roll dither dropped stage 2 from 75 to 33 percent at n = 60 (`--v1 stage --n 60 --procs 3 --weld 0.001 --out t_st2_roll.json`).

## MEASUREMENTS (round 2)

1. Group ablation, weld 0.001, n = 40 per group, 3 processes (`python scripts/capture_region.py --v1 stage --stage 2 --controllers expert --n 40 --procs 3 --weld 0.001 --log-ticks --ablate all,none,latency,gains,sensing,contact,female,friction --out abl_w001.json`), before the settle wait. Seat rate: all 70.0 [54.6, 81.9]; none 100 [91.2, 100]; latency 97.5; **gains 77.5 [62.5, 87.7]**; sensing 100; contact 100; female 100; friction 100. Only the gains group hurts on its own. In the traces the cause is the housing moving along the row under contact (above), not a wrong standoff or travel band.
2. After the settle wait, n = 40 (`--ablate all,gains --out abl2_w001.json`): gains 87.5 [73.9, 94.5], all 72.5 [57.2, 83.9]. Re-traced gains failures 500003, 500012 and 500028 now seat.
3. **Final, stage 2, n = 400** (`python scripts/capture_region.py --v1 stage --stage 2 --controllers expert --n 400 --procs 3 --weld <w> --out r2_stage2_w<w>.json`):
   - weld 0.001: **seat 73.3 percent [68.7, 77.4]**; detector **FN 10.9 percent [7.8, 15.0]**, **FP (fired while unseated) 4.75 percent [3.1, 7.3]**. Outcomes S 293, F-miss 74, F-jam 26, F-partial 7. 75 of the 107 failures hit the 900-tick limit; 35 failures are F-miss at full depth.
   - weld 0.002: **seat 68.8 percent [64.1, 73.1]**; **FN 10.6 percent [7.4, 14.7]**, **FP 4.75 percent [3.1, 7.3]**.
4. **Final, nominal grid** (slip 0 to 4 mm by tilt 0, 4, 8 deg, n = 20 per cell, `--v1 grid --n 20 --slips 0,1,2,3,4 --tilts 0,4,8 --procs 3 --weld <w>`):
   - weld 0.001: seat 97.7 percent [95.3, 98.9]; **FN 0.68 percent [0.19, 2.45]**, **FP 0.33 percent [0.06, 1.86]**.
   - weld 0.002: seat 99.7 percent [98.1, 99.9]; **FN 0.0 percent [0, 1.27]**, **FP 0.33 percent [0.06, 1.86]**.
5. Every one of the 19 stage-2 detector false positives (weld 0.001) is an F-miss at 5.9 to 6.0 mm depth. Two checked (scratch pitch.py, seeds 500120 and 500128): the pin row is seated one pitch off along the row. Pin tips sit at x = 7.60, 5.06, 2.52, -0.02, -2.56 mm against cavities at -5.08 to 5.08 mm, so one pin is outside the housing end. Axially this is a full seat, and the detector cannot tell it apart.

## Pass criteria, round 2

- Expert at least 90 percent at stage 2, n = 400: **FAIL**, 73.3 percent (up from 62.8 percent on main at weld 0.001).
- Detector FN at most 5 percent and FP at most 1 percent on both welds: **PASS on the nominal grid** (FN 0.68 and 0.0 percent, FP 0.33 and 0.33 percent); **FAIL at stage 2** (FN 10.9 and 10.6 percent, FP 4.75 and 4.75 percent).

## CONCERNS (round 2)

- The remaining stage-2 losses are all about where the housing sits along the row, and the right arm's sensors cannot see it. The left (fixture) arm's soft wrist lets the housing slide about 1.5 to 2.5 mm along the row under contact. This causes the timeouts (probes land on the face while the across-row offset is right), and it causes the off-by-one-pitch seats, which also make up every detector false positive. No threshold on travel or torque separates an off-by-one seat from a true seat.
- I found no adaptive standoff or travel-band fix to make: the face onset stayed at 4.2 to 5.3 mm of travel across the gain draws, so the priors are not what breaks.
- The hold now keeps a seated header at about 4 Nm. It no longer creeps up to 6 to 7 Nm, so agent B's labels and rewards see a different held force from round 1.

## OPEN QUESTIONS FOR SHIVEN (round 2)

1. Is the housing on the cell held as softly along the row as the left arm's fitted wrist gains make it here (about 0.45 N/mm at the face)? If the real fixture is stiffer (a nest or a stiffer hold), the left-arm gains or a fixture constraint in `scene.py` should change. That is not my file, and it is the main lever on the stage-2 rate and on detector FP.
2. Should an off-by-one-pitch seat count as F-miss? It is axially a full seat with one pin outside the housing. If the real housing's end walls make it impossible, the bench geometry is too permissive.
3. To catch off-by-one seats, the detector needs a signal it does not have: a wrist-camera check of the row ends, or a continuity test on the real cell. Which one should come next?
