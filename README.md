# FR3 robotic FFC insertion research

**[Watch the experiments online](https://lakshya-asu.github.io/ffc-insertion-research/)** — full videos, stage clips and measured checks.


**[Open the live lab](https://lakshya-asu.github.io/ffc-insertion-research/live/)** — timestamped cell views, experiment progress, engineering decisions and a place to collect discussion notes. [Operation details](LAB_LIVE.md).

**Current task:** Pi 4 Model B + Camera Module 3, with a dimensioned 200 mm ribbon. See the [refined hardware](https://lakshya-asu.github.io/ffc-insertion-research/hardware/) and [new RGB perception experiment](https://lakshya-asu.github.io/ffc-insertion-research/pi-perception/). New perception runs on camera images in an isolated worker. Real-camera qualification and contact mechanics remain open; robot motion is disabled. The manipulation results below are historical generic-scene baselines using privileged simulator state.

A runnable Isaac Sim experiment for picking a ribbon cable off a desk, transferring it from suction to a mechanical pinch, and attempting insertion into an open PCB connector. The project compares direct transfer in the tool with placement and regrasp on a passive fixture.

The Docker runtime, official FR3 model, physical workcell, two-actuator tool, cable models, force gates and stage-video recorder are implemented. **Both direct and passive-fixture desk-to-open-slot sequences pass their independent geometric and grip audits.** Both tips reach 3.801 mm depth; maximum sampled pre-insertion grip drift is 1.32 µm direct and 1.93 µm with the fixture. These are single deterministic privileged-state trials, not reliability estimates. Electrical continuity, latch closure and hardware transfer are not established.

Watch [the qualified direct run](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e040-direct-pgs/experiment.mp4), read [the morning report](MORNING_REPORT.md), or open [the experiment video index](https://lakshya-asu.github.io/ffc-insertion-research/) for actual rendered trials and separate clips for each stage. Read [ISAAC.md](ISAAC.md) for execution and model limits, [experiments/005-numerical-validation.md](experiments/005-numerical-validation.md) for numerical checks, and [task-specification.md](task-specification.md) for the broader research objective.

## Run

Requires an NVIDIA RTX-capable GPU, a compatible driver, Docker and NVIDIA Container Toolkit. The tested machine has an RTX 5080 with 16 GB VRAM. Run from this directory:

```bash
python3 scripts/fetch_fr3_isaac.py
./scripts/docker_experiment.sh build
./scripts/docker_experiment.sh smoke --output /workspace/outputs/my-smoke
./scripts/docker_experiment.sh direct --output /workspace/outputs/my-direct
./scripts/docker_experiment.sh assembly --output /workspace/outputs/my-fixture
```

Use a **new output directory for every run** to preserve its source/configuration snapshot and results. Refined cable physics is intentionally slow; these are offline research experiments. The image is based on NVIDIA Isaac Sim 6.1.0 pinned by digest. Compose supplies GPU access and persistent shader caches without host networking or robot devices.

Each completed run includes a USD scene, results, source hashes, trajectory, contact logs, camera images, full MP4 and per-stage clips. Failures are retained with their measured cause. These are scripted experiments using exact simulated state, not vision-based autonomous manipulation.

## Development checks

```bash
uv sync
uv run pytest -q
uv run ruff check src scripts tests
./scripts/docker_experiment.sh verify outputs/my-direct
uv run python scripts/render_report.py
```

`check_artifacts.py` checks decoded videos and recorded data; it reports the experiment's separate PASS/FAIL status. The seventeen tests verify geometry/kinematics utilities, preload-preserving command transitions, full-tip seating acceptance, and physical suction-anchor placement; full physics validation runs in Isaac Sim.

The earlier MuJoCo endpoint study remains available through `scripts/evaluate_geometry.py` and [E001](experiments/001-workcell-geometry.md). Its model and static-clearance results are separate from the vendor FR3 v2.1 Isaac workcell.

## Sensor-driven architecture

[Interactive architecture and research guide](https://lakshya-asu.github.io/ffc-insertion-research/architecture/) explains the sensors, cable/PCB research, control and learning milestones. [SENSOR_FIRST.md](SENSOR_FIRST.md) records current implementation limits and gates. The historical full-suite runner now requires `--historical-baseline`; it uses privileged simulator state and is not a deployable skill.

## Selected hardware and second task

The first task now uses **Pi 4 Model B + Camera Module 3 Standard + the 200 mm Standard-Standard cable**. [Inspect the CAD renders and roadmap](https://lakshya-asu.github.io/ffc-insertion-research/hardware/) or read [the import and reproduction notes](experiments/009-raspberry-pi-cad.md). The assets are loaded in a separate static FR3 workcell; appearance, connector mechanics and calibrated cable deformation are not yet training-ready.

Stage 2 is a **phone-style press-on FPC-to-board connector**, with a representative research coupon to be selected and measured separately. Neither task uses exact simulator state for new control, critic inputs, stage transitions or rewards.

The [fixed-camera prototype](experiments/008-fixed-camera-perception.md) exercises image ingress and isolated inference on the earlier generic scene. Its endpoint gate failed; it is infrastructure evidence, not a qualified Raspberry Pi perception model.
