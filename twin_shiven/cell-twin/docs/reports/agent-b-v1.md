# Agent B report, training build v1

Branch `feat/training-v1-b`, worktree `~/Professional/cell-twin-wt-b`. Agent A's files at the end: `bench.py`,
`controllers.py` and `training_config.py` as merged into `main` (154ba88 and later, including the coordinator's
fixes 7cbbe64 grasp weld 0.001 s, 6b5a266 MAX_TICKS 900, c737761 expert tick limit 900), taken by `git merge main`.
Torque noise: `out/torque_noise.json` copied from the main checkout.

## CHANGES

- `cell_twin/env.py` (B1). `InsertionEnv`: `reset(seed, randomisation=None, curriculum_stage=1)` and
  `step(action)` returning obs, reward, terminated, truncated, info.
  - Observation: the last 16 rows of `OBS_LAYOUT` (24 per tick), flattened oldest first, plus one extra entry, tau_th
    (Nm, drawn per episode in 4 to 6). The observation has OBS_DIM + 1 = 385 entries.
  - The bench is built with `rand=` (agent A's `Bench.apply` before reset). Torque latency and action latency live in
    the bench. The env delays only the non-torque channels by `obs_latency_ticks`. tau_ext in the row = the
    (bench-delayed) measured torque minus the model torque of the same delayed tick. Torque dropout zeroes that channel.
  - Model torque = `qfrc_bias - qfrc_gravcomp` at the measured joints. The Menagerie YAM cancels gravity through body
    gravcomp, so the actuator force carries no gravity. With `qfrc_bias` alone, tau_ext sat at about 10 Nm at rest.
  - Contact onset comes from the seat detector's `ContactMonitor` (the controllers' rule). The env falls back to the
    0.6 Nm rise rule only if controllers has no `SeatDetector`. The first onset fixes the pose reference: the start
    pose is the reference before onset, the onset pose after it.
  - Seat detector: `controllers.SeatDetector(bench)`, stepped once per tick with the env's torque reading. The stall
    stand-in remains for branches without it.
  - Reward (report 2.3, option B):
    - +10 on the detector, then terminate.
    - +1000 per metre of new max travel since onset, capped at INSERT_DEPTH, so +6 at most.
    - -0.01 per tick.
    - -0.05 x max(0, norm(tau_ext) - tau_th).
    - -5 and terminate on over-force (norm(tau_ext) > 12 Nm) or over-travel (travel since onset > SEAT_BAND[1] + 2 mm
      = 18 mm).
    - -0.01 x norm(a_t - a_{t-1}).
    - Truncate at MAX_TICKS = 900. The coordinator changed it from the spec's 600.
  - `info["truth"]`: depth, pins_in, worst, outcome and the sampled pick error. The env's own decisions never read it.
  - `ExpertProxy` + `run_controller` drive a monolithic Bench controller (align_by_contact, push_only) tick by tick.
    Each step_tool call is split into bounded ticks, and the proxy's `seated()` is the detector, not truth.
  - `SensorView` + `run_policy` drive a per-tick policy (`controllers.Expert`). The view exposes only `torques()`,
    `tool_pose()` and `tool_travel()`. DART noise is added per tick and the clean action is recorded.
- `cell_twin/episode_writer.py` (B2). For each episode it writes four files:
  - `<id>.npz` with obs_row, obs_index, q, q_cmd, tau, tau_model, tau_ext, action, clean_action, applied_action,
    reward, onset, detector, travel and tau_th.
  - `<id>.json` header: randomisation, stage, controller, noise, seed, tau_th, onset tick, end reason, bench git hash
    and detector name. It is written last, so a header means the episode is complete.
  - A separate `<id>.truth.npz`: per-tick depth, pins_in, worst, header-housing contact count and normal force, the
    centre pin lateral position, and the outcome.
  - Optional `<id>.frames.npz`: the three cameras at 320x240 every k ticks (`--frames k`). Checked on 2 episodes:
    7 frames per camera at k = 10.
- `scripts/collect_expert.py` (B3). It runs on a multiprocessing spawn pool, and episode i uses seed seed0 + i.
  - Resumable: an episode is skipped when its header exists.
  - The default controller is `expert` (`controllers.Expert`); `align` and `push` also work.
  - An exception is recorded as outcome X with its traceback in the header.
- `scripts/label_contact.py` (B4). Writes per-row labels `<id>.labels.npy`. The rules are checked in this order, and
  the first that matches wins:
  1. seated (`Bench.seated` rule)
  2. free (no header-housing contact)
  3. beside (worst pin outside the footprint, the F-miss rule)
  4. surface (depth at or above the chamfer bottom, 0.4 mm)
  5. wedged (past the chamfer, |depth change| < 0.02 mm per tick for 5 ticks)
  6. entry
- `cell_twin/datasets.py` and `cell_twin/models.py` (B5).
  - Datasets: lazy windows (row indices, not copies), an episode-level 80/20 split, and per-channel row statistics.
  - `TorqueMLP`: 385 inputs, 3 x 256, 5 outputs with tanh.
  - `ContactCNN`: 24 channels x 16 ticks, 3 Conv1d layers (64 channels, kernel 3), global average pool, 6 classes.
    The normalisation is stored in buffers, so both models take raw env observations.
- `scripts/train_bc.py`: L1 on the clean action, 20 percent of episodes held out, early stop (patience 6). Writes
  `out/models/<name>.pt` and `.json` (per-epoch losses plus the zero-action baseline).
- `scripts/train_classifier.py`: cross entropy weighted by 1/sqrt(class frequency). Reports on the held-out episodes:
  - the confusion matrix
  - per-tick seat FP and FN
  - episode-level FP and FN of the report 4.2 rule (P(seated) > 0.9 for 3 ticks)
  - terminal-state FP and FN

  All with Wilson 95 percent intervals.
- `scripts/eval_policy.py` (B6). Policies: expert, align, push_only or bc.
  - Capture grid: slip {0..5} mm x tilt {0..8 step 2} deg, n per cell, with a random sign. `--grid-dr full` draws the
    stage 3 randomisation and overrides the pick error; `nominal` uses the default Randomisation, as A6 does.
  - Stage 3: n episodes from `Randomisation.sample(rng, 3)`.
  - Grading is truth only, after the episode. Seat rates come with Wilson intervals, plus the detector FP rate on the
    policy's own rollouts. Writes `out/eval_<policy>[_tag].json`.

## THINGS I DID NOT TOUCH

- `cell_twin/bench.py`, `cell_twin/controllers.py`, `cell_twin/training_config.py`: agent A's. I took them from A's
  branch and later from `main`, without editing them.
- `cell_twin/scene.py`, `app/`, `scripts/register_*`, `scripts/capture_region.py`, `scripts/throughput.py`.
- The worktree's `inputs/nuc/episodes/objects` and `inputs/nuc/data/camera-setup` are symlinks to the main checkout.
  Both are untracked and not committed.

## MEASUREMENTS

1. Smoke test before any change, on the bare bench (a6adc3e). `Bench(seated_state, PickError(skew_deg=4,
   lateral_mm=(0,1.0)), rng=default_rng(0))` + `align_by_contact(rng=default_rng(1))` gave F-miss at -1.69 mm, not
   S near 6 mm. The result depends on the seed:

   | skew (deg) | lateral (mm) | seed | outcome | depth (mm) |
   |---|---|---|---|---|
   | 4 | 1 | 1 | F-miss | 5.97 |
   | 4 | 1 | 2 | S | 5.62 |
   | 4 | 0 | 0 | S | 5.44 |
   | 0 | 0 | 0 | F-miss | 4.74 |

   Run: scratchpad `smoke_sweep.py`, same constructor with seeds 0 to 2.
   - Root cause seen in a per-tick trace: the clean first entry reached 3.4 mm depth with a torque rise of only 0.46 Nm,
     under the 0.6 Nm onset rule. align_by_contact then hit its 7.5 mm approach budget, backed out and spiralled into a
     jam.
   - Agent A's final expert replaced this rule with the matched-filter ContactMonitor.
2. Step 2 check: the expert through the env, stage 2, full randomisation, no noise, 1 process.

   `python scripts/collect_expert.py --n 20 --procs 1 --stage 2 --noise 0 --controller expert --run check_expert_s2`

   - **15 of 20 seated (75 percent, Wilson [53.1, 88.8])**; the other 5 were F-miss.
   - Agent A's bare-bench figure is 64.3 percent [59.4, 68.8] at n = 400 (600 ticks), so the intervals overlap.
   - 93 s wall for 5600 ticks: 60 ticks/s, 12.9 episodes/min on one process.
   - Ending of the 20 episodes (per-episode table from scratchpad `ana.py check_expert_s2`):
     - 17 ended `controller_returned`: the expert decided it was done.
     - 2 were truncated at 900 ticks.
     - 1 ended on the env's seat detector.
   - So the +10 seat reward fired in only 1 of the 15 truth-seated episodes, because the expert stops itself first. See
     CONCERNS.
3. 500-episode end to end. The coordinator ran it from this branch before the merge, with `align_by_contact` and the
   stall stand-in detector, 4 processes, stage 2, noise 0.3. Log: `out/e2e_v1.log`.

   | step | wall time | result |
   |---|---|---|
   | collect | 5131 s (saturated machine; load average 100 to 160 at the time, see note) | 5.8 episodes/min, 80093 ticks; 9 S, 475 F-miss, 13 F-jam, 3 F-partial |
   | label | 1 s | 80093 ticks: beside 18.2 %, surface 0.8 %, entry 0.6 %, wedged 0.1 %, seated 0.3 %, the rest free |
   | train BC | 4.3 s | best held-out L1 0.042 against 0.167 for the zero action (400 train / 100 held-out episodes, 63973 / 16120 samples, best epoch 15) |
   | train classifier | 176 s | held-out accuracy 0.843; see below |
   | eval BC (small: grid n = 4 per cell, stage 3 n = 40, 4 processes) | 142 s | grid 16/120 = 13.3 % [8.4, 20.6]; stage 3 4/40 = 10.0 % [4.0, 23.1] |
   | **total** | **5455 s** | |

   - The classifier never predicts seated. Seat FN on held-out ticks = 150 of 150 (100 percent), episode FN 5 of 5,
     FP 0 of 95 (upper bound 3.9 percent). Training had only 82 seated ticks out of 64 k.
   - Machine load during this work: `uptime` showed load averages of 107 to 163 at 03:43 and 04:21, from other
     sessions' vitest workers. The classifier smoke on 40 episodes took 20.7 s of CPU and 4 min 24 s of wall time (8
     percent CPU). The 5131 s collection is a statement about the machine at that time, not about the code. The
     unloaded rate in item 2 (12.9 episodes/min per process) predicts about 10 min for 500 expert episodes on 4
     processes.
4. tau_ext at rest, nominal bench, seed 0: norm 10.0 Nm with `qfrc_bias` as the model torque, 0.11 to 0.22 Nm with
   `qfrc_bias - qfrc_gravcomp`. Script: scratchpad `env_try.py`, 6 episodes.

## Pass criteria against the spec

| criterion | status | evidence |
|---|---|---|
| BC within 5 points of the expert on the grid | NOT MEASURED with the final expert | The BC above learned align_by_contact, a 2 to 5 percent teacher; the grid eval was n = 4. Needs a collection with `--controller expert` and the grid at n = 40 for expert and bc, which needs more than 2 processes. |
| Classifier seat FP < 1 %, FN < 5 % | FAIL | FP 0/95 episodes (upper bound 3.9 %), FN 100 % on the 500-episode run. Too few seated ticks, from a weak teacher. |
| End to end on 500 episodes under 10 minutes | FAIL as measured | 5455 s on a saturated machine with align_by_contact. Label, BC and eval are seconds to minutes; collection dominates. The unloaded estimate is about 10 min for collection alone on 4 processes (item 2), so it is borderline even unloaded. Needs a rerun with the expert once the CPU is free. |

## CONCERNS

- The env's seat reward rarely fires on the expert's own rollouts: once in 15 truth seats. The expert's internal done
  rule (4 ticks of band + stall + level, or 45 hold ticks) ends the episode before the detector instance in the env
  confirms, apparently because the env detector also requires `popped`.
  - For RL this means seats often get no +10.
  - Two options: keep stepping zero actions for up to N ticks after the expert finishes, or have the expert's done
    flag wait for the detector.
  - I did not change either, because one is in agent A's code and the other is a reward design decision.
- The over-travel bound is 18 mm since onset because A's SEAT_BAND reaches 16 mm. That bound is loose for a
  safety abort.
- The header json records the sampled pick error inside `randomisation` (spec rule 4). Rule 1 calls it truth.
  `datasets.py` never reads headers for model inputs, but anything that does should drop those fields.
- tau_model uses MuJoCo's arm model at the measured joints. On the real cell this needs the YAM's own gravity model,
  and whether the recorded effort includes the gravity feedforward decides which formula is right.
- DART noise is applied by the env runner (`run_policy`), not `controllers.noisy`. It is equivalent: clean + N(0,
  sigma), clipped, with the clean action recorded.
- Building a Bench per episode costs about 0.8 s, because the pick error is compiled into the model. That is about 15
  percent of a 5 s episode.

## OPEN QUESTIONS FOR SHIVEN

1. Should the episode continue after the expert declares done until the env's detector fires (or a short timeout), so
   the +10 seat reward and the detector FP/FN on rollouts are measured on every seat? Or should the expert's done wait
   for the detector?
2. Is the 18 mm over-travel abort acceptable, or should the abort use the expert's FAR_TRAVEL logic from the start
   instead of travel since onset?
3. May I rerun the 500-episode end to end with `--controller expert` and the n = 40 grid for expert and bc when the
   CPU is free (4 processes, about 10 min collect, about 1 h for the two grids)? That is the run that decides the
   BC-gap and classifier criteria.
4. tau_model: does the real YAM effort reading include the gravity feedforward? That decides whether tau_ext is
   `tau - (bias - gravcomp)` (as built) or `tau - bias`.
