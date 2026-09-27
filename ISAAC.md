# Isaac Sim experiment

This is a simulation-only project. The requested Docker installation uses NVIDIA's official Isaac Sim 6.1.0 Linux AMD64 image pinned by digest. It does not connect to a physical robot or publish ROS topics.

## Reproduce

```bash
python3 scripts/fetch_fr3_isaac.py
./scripts/docker_experiment.sh build
./scripts/docker_experiment.sh smoke --output /workspace/outputs/my-smoke
./scripts/docker_experiment.sh direct --output /workspace/outputs/my-direct
./scripts/docker_experiment.sh assembly --output /workspace/outputs/my-fixture
./scripts/docker_experiment.sh cantilever --config /workspace/config/selected-static.json --cantilever-length .15 --steps 16000 --output /workspace/outputs/my-cantilever
./scripts/docker_experiment.sh native-cantilever --output /workspace/outputs/my-native-cantilever
```

`assembly` selects the passive-fixture strategy; `direct` transfers from suction to pinch in the air. `grasp` is an isolated acquisition/lift diagnostic initialized at the pickup pose and does not test the approach. `deformable` exercises the native shell independently; full handling currently uses the segmented cable.

Always choose a fresh output directory. Main handling configuration: `config/isaac-workcell.json`. Native shell: `config/native-shell.json`. Read the configuration copied into each run's `source/config.json` to establish what that run actually used. Editing a configuration does not retroactively change old experiments.

Compose mounts the project at `/workspace`. The image also bundles its source and robot assets; a standalone run with only an output-directory bind mount was verified. Cold renderer initialization can take several minutes before the first valid camera frame. Caches use named Docker volumes. GPU access and 4 GB shared memory are supplied. No privileged mode, host network or physical robot devices are needed. NVIDIA EULA acceptance flags are set for the requested installation; explicit Kit privacy flags disable anonymous telemetry, usage, performance and personalization reporting. The absence of the telemetry transmitter was checked in a live container.

## Models and assumptions

- **FR3:** vendor FR3 v2.1 USD release v0.1.0, SHA256 checked. Composed vendor joint frames define new kinematics. The earlier MuJoCo asset is a different variant.
- **Handling cable:** 150 × 8 × 0.3 mm, assumed mass 0.7 g, 30 rigid segments with D6 spring joints and two end segments stiffened. The out-of-plane bend stiffness corresponds to a nominal homogeneous 3 GPa beam. Equal D6 swing gains mean twist and in-plane bending are not independently modeled. Material, friction and the thickness profile need physical calibration.
- **Native cable comparison:** triangular finite-thickness surface FEM with explicit mass and effective homogeneous elasticity. It bends, stretches and twists, but does not resolve copper/PET anisotropy or end stiffeners. Mesh, timestep and iteration sweeps show substantial numerical sensitivity.
- **Tool:** fixed upper shoe, two 3 mm suction patches, lateral lower-jaw deployment and a separate normal clamp. Both prismatic actuators have finite force limits. The lower jaw parks above the pickup plane. Offset aluminium brackets contribute to calculated mass/inertia. This is a concept, not a manufactured gripper or a selected commercial actuator.
- **Suction:** proximity/normal-gated, compliant breakable D6 attachments, 30 kPa assumed vacuum and 0.212 N ideal force limit per patch. Relative separation is monitored. This does not solve pressure, leakage or adhesion. The cable anchors lie on its actual surface, and the initial capture gap is resolved by the compliant attachment. All six free-joint drive stiffnesses, dampings and force limits are zeroed before evaluating pinch-only retention. Cable/shoe contact remains enabled.
- **Connector:** FH12-sized envelope, slot, backstop and force-limited hinged latch. Interior and slot tolerances remain design assumptions; vendor STEP access was unavailable. The latch starts open and is not closed by the baseline. Terminal springs are not resolved; the qualified direct trial enters the clearance slot without cable/connector contact. No electrical or damage model is claimed.
- **Fixture:** raised passive shelf, underside access for the lower jaw and a rear guide. Cable retention comes from contact; there is no hidden attachment.
- **Cameras:** actual RTX overview and tool-following detail videos, plus overhead and connector macro captures. Optics and lighting are proposed, without calibrated sensor noise. The controller does not infer state from these images.

## Controller and validation

The baseline begins in an overhead ready pose with the cable still ungrasped on the desk, and plans smooth, continuous Cartesian waypoints using the vendor kinematic chain. The fixture branch lands the free tail before laying down the held end. Before connector alignment, approach and insertion, it measures the simulated tip pose and corrects the arm target; this is privileged feedback, not camera-based estimation. The arm uses an assumed position servo with gravity and velocity feedforward plus bounded integral correction. It is not an identified FR3 firmware/controller model. The selected segmented-cable profile uses PGS with 255 position iterations and four velocity iterations. It commands at 1 kHz while integrating physics at 8 kHz. The awake full-span benchmark must agree with the independent static reference within 1%.

Gates check pickup/lift, fixture or pinch retention, tool tracking, geometric seating of all four tip corners (each deeper than 3.7 mm in the nominal 4 mm slot), unintended robot/environment contact, robot self-contact, unintended tool self-contact, broken attachments and excessive cable/connector contact load. Rigid contact force is estimated from summed contact impulse magnitudes divided by the actual timestep. An unchanged jaw/deployment target is preserved across phase boundaries to maintain contact preload. Its 0.5 N insertion stop is a research assumption, not a certified hardware operating limit. Earlier TGS trials showed large numerical contact spikes; the selected PGS transfer peaked at 0.252 N per cable/tool contact pair. These estimates still require physical calibration before hardware force limits or damage predictions can be derived. Native deformable contact force is not validated through this rigid-contact interface.

Every physics step is counted by a callback. Render calls do not advance simulation time. Source/configuration snapshots and hashes precede simulator startup. Each video frame comes from the live simulated state. Contact logs are sampled at 100 Hz, per-pair peaks are retained, and force gates run at every physics step. Trajectories are sampled at 10 Hz.

A results `PASS` means only that run's declared checks passed. Inspect `assembly_success_tested`, `grasp_benchmark_only`, `cantilever`, `timestep_audit` and the episode seating event. Older development runs without clock audits are explicitly labeled in the video index. Out-of-range native iteration requests are excluded from numerical comparisons.

## Reproducing the analysis

```bash
uv sync
uv run python scripts/plot_benchmarks.py
uv run python scripts/compare_segment_benchmarks.py
./scripts/docker_experiment.sh verify outputs/e039-pgs-transfer
uv run python scripts/render_report.py
```

See [E005](experiments/005-numerical-validation.md). Numerical agreement with a nominal beam/chain checks the solver and discretization; it does not identify real cable properties. Physical acceptance still requires measured mass/thickness, bending/torsion, friction, insertion force, latch torque, continuity and damage inspection.

## Primary references

- [NVIDIA container setup](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_container.html)
- [Native deformables and implementation limits](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.1/dev_guide/deformables/deformable_bodies.html)
- [Franka vendor assets](https://github.com/frankarobotics/franka_simulation)
- [Hirose connector reference](https://www.hirose.com/product/p/CL0586-0523-6-55)
- [USD drive unit conversions](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/population/PhysicsDriveAPI.html)

The installed PhysX extension is 110.3.2. Its bundled schemas and examples were inspected when the corresponding public API documentation was unavailable.

Run `uv run python scripts/audit_grip.py outputs/my-direct` followed by `uv run python scripts/audit_seating.py outputs/my-direct` for independent grip and final-scene checks. The seating audit requires the grip audit to pass, rejects recorded unintended tool self-contact, and preserves the original experiment result. Docker-only analysis can use `docker compose run --rm --entrypoint /workspace/scripts/usd_python.sh experiment scripts/audit_grip.py outputs/my-direct` and the same command with `audit_seating.py`.

E040 direct completed all 16 phases in 41.76 simulated seconds, seating the full tip 3.801 mm deep with 1.32 µm maximum sampled pre-insertion grip drift. All 334,080 physics callbacks, 1,253 video frames and 16 stage clips verified. There were no fixture contacts. See [the morning report](MORNING_REPORT.md) for the selected results and limits.
