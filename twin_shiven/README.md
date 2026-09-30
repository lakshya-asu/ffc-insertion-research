# ffc-twin

Simulation twin of the last 10 mm of ribbon-cable insertion (Raspberry Pi camera cable into a Pi socket), built to train and score sensor-only insertion policies on a Mac in MuJoCo and to cross-evaluate them in Isaac.

Two board profiles: `pi4` (top entry, TE 1-1734248-5 reference, 15-way 16 mm cable) and `zero` (Pi Zero 2 W side entry, Molex 54548 reference matched to Lakshya's experiment 045 socket with its 22 sprung contacts, 22-way 11.5 mm cable). Two grip options (`tape`: short jaws on the stiffener; `body`: Lakshya's 6 mm pads 12 mm behind the edge) and two arm classes (`--dof 6` six-axis, `--dof 4` SCARA). `make_spec(board, grip, dof)` in `spec.py` is the one entry point.

Companion to Lakshya's [ffc-insertion-research](https://github.com/lakshya-asu/ffc-insertion-research); a copy lives on its `shiven/ffc-twin` branch under `twin_shiven/`.

## Layout

- `ffc_twin/spec.py`: every number with its source; `Spec().placeholders()` lists the guesses that measurements must replace.
- `ffc_twin/mjcf.py`: MuJoCo scene from the spec.
- `ffc_twin/usd.py`: the same scene as USD (tool articulation with six drives, rigid header, tail as an articulated chain, Factory-style solver settings).
- `scripts/emit.py`: writes `out/ffc_twin.xml`, `out/ffc_twin.usda`, `out/ffc_twin_spec.json`.
- `tests/`: both emitters load; the geometry matches the spec in both; a scripted push seats the cable in MuJoCo from zero error.

## Run

```bash
python3 -m venv .venv && ./.venv/bin/pip install "mujoco==3.12.0" numpy pytest "usd-core==26.8"
./.venv/bin/pytest -q
./.venv/bin/python scripts/emit.py
```

MuJoCo stays on 3.12.0 (Lakshya's pin; 3.13 changed flex integration).

## Frames

Connector frame C at the slot mouth: +x into the connector, +y across the cable width, +z the cable surface normal (Lakshya's task contract). The tool starts with the tip 1.5 mm before the mouth.

## Environment, detector, expert

- `ffc_twin/env.py`: `TwinEnv` with reset levels L0/L1, domain randomization (friction, stiffness, tool gains), simulated sensors (delayed noisy tip estimate with proprioceptive dead reckoning when the mouth is occluded; first-order wrench), the observation packet, a sensor-only reward, and the episode log with truth kept in a separate field for offline grading.
- `ffc_twin/detector.py`: the observable seating rule (travel since the estimated mouth crossing, stall force, estimated pose in band). Graded by `scripts/grade_detector.py`.
- `ffc_twin/expert.py`: the sensor-driven scripted expert (align, approach, advance, retract and shift).
- `scripts/capture_region.py`: the capture-region sweep with Wilson intervals; writes JSON and SVG to `out/`.

```bash
./.venv/bin/python scripts/grade_detector.py 60 zero good,poor,lakshya
./.venv/bin/python scripts/capture_region.py --n 200 --board zero --grip tape --dof 6 --estimator good --controller expert --level L1 --procs 8
```

Estimator presets: `good` (lit, focused), `poor` (4x pixel noise, mismatched pad), `lakshya` (his experiment 046 stereo error: 0.03 mm depth, 0.03 mm lateral, 0.088 mm height bias).

## Learned controller (rung 1) and Isaac replay

- `ffc_twin/policy.py`: behaviour cloning of the expert on the last 8 packets; `scripts/train_bc.py` trains it (PyTorch, MPS on the Mac); `capture_region.py --controller bc --model out/bc_...` scores it on the same grid.
- `scripts/collect_expert.py --board zero --grip tape --dof 6`: the expert's training set, with corrective episodes.
- `scripts/export_replay.py`: one run as a USD timeline (every geom's pose per tick, physics disabled) for Isaac's renderer; mirrors Lakshya's recorded-state replays.

```bash
./.venv/bin/pip install torch
./.venv/bin/python scripts/collect_expert.py --episodes 10000 --shard 500 --procs 6 --board zero
./.venv/bin/python scripts/train_bc.py --data out/dataset_expert_zero_tape_6dof_good_L1 --out out/bc_zero_tape_6dof --epochs 40
./.venv/bin/python scripts/capture_region.py --n 20 --board zero --controller bc --model out/bc_zero_tape_6dof --procs 6
./.venv/bin/python scripts/export_replay.py --board zero --grip tape --seed 3
```

## Status

Rungs 0 and 1 done on both boards (30 Sep 2026). Percent seated under 3 N, mean over the 63-cell L1 grid:

| Board | Controller | Grip, arm | Estimator | Runs per cell | Mean |
|---|---|---|---|---|---|
| Pi 4 | expert | tape, 6 axes | good / poor / none (push) | 200 | 94.6 / 94.1 / 0.0 |
| Pi 4 | learned (bc) | tape, 6 axes | good / poor | 20 | 99.0 / 98.7 |
| Zero | expert | tape, 6 axes | good / lakshya | 20 | 96.7 / 89.4 |
| Zero | expert | body (12 mm), 6 axes | good | 20 | 76.6 |
| Zero | expert | tape, 4 axes (SCARA) | good, L1 / L0 | 20 | 1.7 / 17.1 |
| Zero | expert | body, 4 axes | good, L0 | 20 | 35.0 |
| Zero | learned (bc) | tape, 6 axes | good / lakshya | 20 | 97.1 / 88.5 |

Seated-rule grade (false seated / missed): Pi 4 good 1/61, 0/59; Zero good 1/45, 1/45; poor 2/45, 2/45; lakshya 5/49, 1/41. Next: the Isaac cross-check with Lakshya's renderer and DINOv3 in the loop, then rung 2 (PPO from the bc warm start, force-limit input) on a GPU under MuJoCo Warp.
