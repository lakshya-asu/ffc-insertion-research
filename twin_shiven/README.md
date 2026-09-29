# ffc-twin

Simulation twin of the last 10 mm of ribbon-cable insertion (Raspberry Pi camera cable into the Pi 4 CSI socket), built to train and score sensor-only insertion policies on a Mac in MuJoCo and to cross-evaluate them in Isaac Lab.

Companion to Lakshya's [ffc-insertion-research](https://github.com/lakshya-asu/ffc-insertion-research). Nothing here is committed to that repository; files are handed over.

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
./.venv/bin/python scripts/grade_detector.py 60
./.venv/bin/python scripts/capture_region.py --n 200 --estimator good --controller expert --level L1 --procs 8
```

## Status

S1 to S4 built with researched sensor models (`ffc_twin/sensor_models.py`) and an assumptions register (`ASSUMPTIONS.md`, from `scripts/assumptions.py`). Seated-rule grade with the good sensing: 1 false positive in 61, 0 misses in 59; poor sensing: 0 and 0. Expert on the L1 grid, 20 runs per cell: 94 percent (good sensing), 95 percent (poor), straight push 0. Next: the 200-run grids, the expert training set, and the first learned controller.
