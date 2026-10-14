# Training build v1: what to build so sim data collection and training can start

Date 2026-10-07. Owner: Shiven. Builders: two dev agents in git worktrees of this repo (`~/Professional/cell-twin`).
Read this file fully before touching code. Read `~/Professional/cell-twin/cell_twin/bench.py`,
`cell_twin/controllers.py`, `cell_twin/scene.py` (constants at the top and the camera block) and
`/Users/shivenagarwal/GamifyAssessment/docs/research/2026-10/contact-policy-training.md` sections 1.2, 2.2, 2.3,
3.2, 4.2 and 7.1. The research report is the evidence; this spec is the contract.

## Rules that bind every file

1. Sensor-only rule. Nothing from simulator truth (true part pose, pin depth, contact pairs, the sampled pick error)
   enters the policy observation, the reward, the classifier input, or any stage switch at run time. Truth is written
   to a separate grading record and used only offline: for grading, for supervised classifier labels, and for the
   bench's one-time reset alignment that already exists in `Bench.reset`.
2. Reward is hand-built from observables (report section 2.3, option B). The learned classifier is never the reward.
3. One action per 30 Hz tick, continuous, bounded, tool frame. No chunking.
4. Every randomised quantity is drawn once per episode from `Randomisation` (below) and recorded in the episode header.
5. No em dashes in any text, docstring or report. Python 3.14, numpy, mujoco 3.12, torch (installed in `.venv`).
6. Each agent edits only the files it owns (listed per agent). Shared constants go in `cell_twin/training_config.py`,
   owned by agent A, created first; agent B imports from it and may ask A (via the report file) for additions.
7. Each agent writes a fixed-format report to `docs/reports/<agent>-v1.md` with sections CHANGES, THINGS I DID NOT
   TOUCH, MEASUREMENTS (numbers with the command that produced them), CONCERNS, OPEN QUESTIONS FOR SHIVEN.
8. Commit on the worktree branch with messages `Feat: ...` or `Fix: ...`. Do not touch `app/` or `scripts/register_*`.

## Shared interface (agent A creates `cell_twin/training_config.py`)

```python
TICK_HZ = 30
WINDOW = 16                      # ticks of history in the observation
ACTION_BOUNDS = dict(lateral_m=0.0003, push_m=0.0005, rot_rad=np.radians(0.5))   # per tick, tanh-bounded
OBS_LAYOUT = [                   # per tick, in this order; the window is the last WINDOW ticks flattened, oldest first
  ("tool_pose_rel_onset", 6),    # xyz (m) + rotvec (rad) of the measured tool pose relative to the pose at contact onset
  ("tau_ext", 6),                # measured torque minus model torque (gravity feedforward + PD on commanded error)
  ("tracking_err", 6),           # q_cmd - q_measured
  ("last_action", 5),            # the previous tick's action in ACTION_BOUNDS units
  ("time_since_onset", 1),       # seconds, 0 before onset
]
OBS_DIM = 24 * WINDOW
ACTION_DIM = 5                   # dx, dy, dz, droll, dpitch in the tool frame (no yaw: the pick error has none)
CONTACT_CLASSES = ["free", "surface", "beside", "entry", "wedged", "seated"]
```

`Randomisation` dataclass (agent A) with these fields and default ranges, all sampled per episode:

| field | range | applies to |
|---|---|---|
| kp_scale[6], kd_scale[6] | proximal 0.9 to 1.1, wrist 0.8 to 1.2 | actuator gains |
| torque_scale[6] | 0.85 to 1.15 | torque sensing |
| torque_bias[6] | proximal +-0.2 Nm, wrist +-0.05 Nm | torque sensing |
| torque_noise_std[6] | from `out/torque_noise.json` (agent A measures it) times 0.5 to 2.0 | torque sensing |
| obs_latency_ticks, act_latency_ticks | 0, 1 or 2 each | env |
| part_friction | 0.2 to 0.8 | plug and socket geoms |
| contact_stiffness_scale | 0.7 to 1.3 (LEAF_SOLREF time constant) | socket contact plates |
| female_tilt_deg | 0 to 2 | housing pick error |
| header_tilt_deg, header_slip_mm | curriculum stage: (0 to 4 deg, 0 to 2 mm), (0 to 8, 0 to 4), (0 to 10, 0 to 6) | header pick error |
| torque_dropout | True in 20 percent of episodes: tau_ext channel zeroed in the observation | env |

Episode outcome labels: `S`, `F-partial`, `F-jam`, `F-miss` (existing `Bench.outcome()`), plus `X` for an exception.

## Agent A: bench, expert, randomisation, torque noise, throughput

Owns: `cell_twin/bench.py`, `cell_twin/controllers.py`, `cell_twin/training_config.py` (new),
`scripts/torque_noise.py` (new), `scripts/throughput.py` (new), `scripts/capture_region.py` (may extend).

Deliverables:

A1. `scripts/torque_noise.py`: from the 23 recorded episodes in `inputs/nuc/episodes/objects/*/frames.npz`
    (keys `observation.state`, `observation.effort` or the effort columns documented in `scripts/replay.py`),
    find static holds (joint speed under 0.002 rad per tick for at least 15 ticks), take the per-joint std of the
    effort in those windows, write `out/torque_noise.json` with per-joint std for both arms and the number of
    windows used. Replace `TORQUE_NOISE_NM` in bench.py with these values (fallback to 0.05 if the file is absent).
A2. `Randomisation` in `training_config.py` and `Bench.apply(randomisation)` that sets gains, torque sensing
    parameters, friction, contact stiffness and latency before `reset`. Torque sensing in `Bench.torques()` must apply
    scale, bias, noise, a 1-tick low-pass and the latency queue. Record the sampled values on `bench.rand`.
A3. `controllers.expert(b, rng)`: the sensor-only scripted expert. Sequence: approach until the torque rise (existing
    rule), then tilt-to-null: estimate the wrist moment direction from `tau_ext` on joints 4 to 6 and rotate about
    the pinch axis in small steps until the moment magnitude stops falling (bounded at 8 deg total), then the existing
    spiral search on stall, push to seat, back off 1 mm and re-tilt if torque exceeds `PUSH_BOUND_NM` with no travel.
    It must read only `b.torques()`, `b.tool_pose()`, `b.tool_travel()` and its own commands. It returns the per-tick
    action it took, in `ACTION_DIM` tool-frame units, so agent B can record it. Also add `controllers.noisy(expert,
    sigma)` that adds Gaussian noise to the expert's action (DART style) while recording the clean action.
A4. Hand-built seat detector `controllers.seat_detector(b)` from observables only: travel since onset within the
    expected seat band, stall (travel rate under 0.1 mm over 8 ticks) and the torque signature (tau_ext along the push
    axis above the seat level). Report its false positive and false negative rate against `Bench.seated()` over at
    least 400 episodes, by outcome class.
A5. `scripts/throughput.py`: episodes per minute per process with full contact, for 1, 4 and 8 processes.
A6. Capture grid for push_only, align_by_contact and expert at n = 40 per cell over slip {0, 1, 2, 3, 4, 5} mm by
    tilt {0, 2, 4, 6, 8} deg, with Wilson intervals, written to `out/capture_v1.json` and summarised in the report.

Pass: expert seat rate at least 90 percent over the stage 2 curriculum range (n at least 400); seat detector FP under
1 percent and FN under 5 percent; throughput measured.

## Agent B: environment, episode writer, datasets, training, evaluation

Owns: `cell_twin/env.py` (new), `cell_twin/episode_writer.py` (new), `cell_twin/datasets.py` (new),
`cell_twin/models.py` (new), `scripts/collect_expert.py`, `scripts/label_contact.py`, `scripts/train_bc.py`,
`scripts/train_classifier.py`, `scripts/eval_policy.py` (all new), `docs/reports/agent-b-v1.md`.

Deliverables:

B1. `InsertionEnv` in `env.py`, gym style (`reset(seed, randomisation=None, curriculum_stage=1)`, `step(action)`
    returning obs, reward, terminated, truncated, info). Observation per `OBS_LAYOUT`, with the latency queues.
    Contact onset is detected from observables (the torque rise rule from controllers.py), never from truth.
    Reward (report 2.3 option B): +10 on the hand-built seat detector (agent A's `seat_detector`, use the existing
    `align_by_contact` stall rule as a stand-in until it lands), +k times the increase in max travel since onset capped
    at seat depth, -0.01 per tick, -beta max(0, norm(tau_ext) - tau_th) with tau_th sampled in 4 to 6 Nm and given to
    the policy as an input (append 1 dim to the observation and note it in the report), -5 and terminate on over-force
    or over-travel, -c norm(a_t - a_{t-1}). Truncate at 600 ticks. `info["truth"]` carries depth, pins_in, outcome and
    the sampled pick error for grading only.
B2. `EpisodeWriter`: one `.npz` per episode under `out/episodes/<run>/<id>.npz` with per-tick arrays (q, q_cmd, tau,
    tau_model, tau_ext, action, clean_action, obs window index, onset flag, detector state) plus a header json
    (randomisation sample, curriculum stage, controller name, seed, bench git hash) and a SEPARATE `<id>.truth.npz`
    (depth, pins_in, contact pair summary per tick, outcome). Optional frames from the three cameras at 320x240
    every k ticks when `--frames k` is passed.
B3. `scripts/collect_expert.py --n 15000 --procs 8 --stage 2 --noise 0.3`: multiprocessing collection with the expert
    (use `align_by_contact` until agent A's `expert` exists, then switch by name), one process per worker, resumable.
B4. `scripts/label_contact.py`: 6-class per-tick labels from the truth files per report 4.2 (surface = pin contacts the
    housing top with depth 0, beside = contact outside the housing, entry = depth increasing past the mouth, wedged =
    depth past the mouth with velocity near 0 for 5 ticks and not seated, seated = within seat tolerance).
B5. `models.py`: `TorqueMLP` (OBS_DIM to 3 x 256 to ACTION_DIM, tanh output) and `ContactCNN` (1-D conv, 3 layers,
    64 channels, kernel 3, global pool, 6-way). `train_bc.py` (L1 loss, 20 percent held out, early stop, writes
    `out/models/bc_v1.pt` and a json of train and held-out loss). `train_classifier.py` (cross entropy with class
    weights, confusion matrix on held-out, seat FP and FN, writes `out/models/clf_v1.pt`).
B6. `scripts/eval_policy.py`: runs a policy (expert, bc, push_only) on the capture grid of agent A's A6 and on
    n = 1200 over the stage 3 range, truth graded, Wilson intervals, writes `out/eval_<policy>.json`.

Pass: a BC student within 5 points of the expert on the grid; classifier seat FP under 1 percent and FN under 5
percent on held-out episodes; the whole pipeline runs end to end on 500 episodes in under 10 minutes on this laptop
(10 cores, 17 GB).

## Not in this build

Residual PPO, the wrist-image variant, real-data co-training, the paired-frame image gate. They come after both
reports are in.

## Rulings after round 1 (2026-10-08, Shiven's standing rules applied by the orchestrator)

1. Grasp weld stays at GRASP_SOLREF 0.001 (the grip is rigid). The seat detector must not depend on the plug popping
   through on a compliant weld; it is being rebuilt from travel band, stall and torque level (agent A round 2).
2. Episode end: the expert declaring itself done is NOT an episode end. The episode ends when the env's seat detector
   fires, on the abort rules, or at 900 ticks. After the expert's done, the env keeps stepping with a zero action for
   up to 30 ticks so the detector can confirm; the +10 reward is paid on the detector, never on the expert's flag.
3. Over-travel abort at 18 mm since onset is accepted as a safety bound for now.
4. Real-cell torque contract: the YAM effort reading INCLUDES the gravity and friction feedforward (the free-hang
   baseline on right joint 2 is about -9.8 Nm in the recordings). The bench's actuator force EXCLUDES gravity
   (gravcomp bodies carry it). So tau_ext on the cell = effort - (gravity model + friction feedforward + PD term),
   while in the bench tau_ext = actuator force - PD term. Both give the same external torque; the deployment code
   must subtract the gravity model on the cell. Agent B's env is correct for the bench; document the cell side in the
   deployment adapter when it is written.
5. The across-row search ordered by contact class replaces the spiral in the expert (the spiral needs ~280 probes to
   reach a 5 mm offset).
