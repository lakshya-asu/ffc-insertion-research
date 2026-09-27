# FR3 / FFC experiment — morning handoff

Completed 27 September 2026: **both the direct and passive-fixture strategies pass the complete desk-pickup → mechanical grasp → geometric insertion sequence.** Independent audits confirm full-tip seating and grip stability; source hashes, physics timing and every video clip verified. These are single deterministic trials using privileged simulated state.

Open the [video index](https://lakshya-asu.github.io/ffc-insertion-research/). It includes completed trials, failures, independent audits and the latest frames of active runs. Individual stage clips sit beside each full MP4.

## Verified outputs

| Experiment | What passed | Evidence |
|---|---|---|
| Direct desk-to-slot sequence | 3.801 mm full-tip depth, 1.32 µm maximum sampled grip drift, no fixture contacts | [Full video](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e040-direct-pgs/experiment.mp4), [16 stage clips](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e040-direct-pgs/stage-clips), [independent seating audit](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e040-direct-pgs/seating-audit.json) |
| Fixture placement, regrasp and insertion | 3.801 mm full-tip depth, 1.93 µm maximum sampled grip drift, physical shelf support | [Full video](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e040-fixture-pgs/experiment.mp4), [23 stage clips](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e040-fixture-pgs/stage-clips), [independent seating audit](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e040-fixture-pgs/seating-audit.json) |
| Corrected workcell/tool stroke test | Both actuator strokes, finite state, collision gates and physics clock | [Video](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e044-pgs-tool-smoke/experiment.mp4), [seven stage clips](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e044-pgs-tool-smoke/stage-clips), [artifact checks](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e044-pgs-tool-smoke/artifact-checks.json) |
| Full-length cable static shape | Declared ±1% agreement with an independent nonlinear chain reference, bounded final pose motion | [Comparison plot](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e043-selected-static/static-shape-comparison.png), [video](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e043-selected-static/experiment.mp4), [results](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e043-selected-static/results.json) |
| Native surface-FEM workcell | Finite cable settling and tool exercise; handling not tested | [Video](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e023-native-workcell/experiment.mp4), [artifact checks](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e023-native-workcell/artifact-checks.json) |
| Suction-to-pinch transfer and raise | Stable contact grip after venting; about 1.1 µm maximum sampled tool-relative tip drift | [Video](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e039-pgs-transfer/experiment.mp4), [grip audit](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e039-pgs-transfer/grip-audit.json) |
| Graceful interruption | Docker stop preserves an explicit ABORTED result, playable video and matching clock audit | [Artifact checks](https://lakshya-asu.github.io/ffc-insertion-research/outputs/e030-stop-check02/artifact-checks.json) |

The clean image includes the official FR3 assets and passes 17 tests. A separate standalone GPU run, with only its output directory bound from the workspace, rendered 30 verified frames and passed its clock/source audit. Cold renderer initialization took about 110 seconds. Tests cover kinematics, numerical references, maintaining actuator preload and rejecting incorrectly oriented or over-inserted cable tips. Source hashes and decoded-media checks accompany the selected results.

## What the handling tests revealed

The earlier suction-to-pinch test retained cable height and initially passed its limited gate. A stronger tool-relative audit found 11.13 mm tip drift, so it is **not a stable-grasp demonstration**. The full direct run reached pickup and venting, then failed as transport began; maximum tip drift was 19.25 mm. Both are retained in the index with the failed independent audit.

Development exposed a zero-clearance guide rod contacting the upper shoe, a controller transition that removed clamp preload, and a release/rigid-contact stability problem. The first two have concrete fixes: 3 mm rod clearance checked before simulation, a tool self-contact gate, and regression-tested preservation of unchanged actuator targets. Those two comparisons failed to stabilize transfer. Further inspection corrected suction anchors that lay in the air gap; a regression test now requires anchors on the cable surface. Those changes alone did not stabilize TGS handling. A PGS solver profile now matches the independent static reference within 0.11% and passes handoff plus a further raise, with 1.1 µm maximum sampled tip drift. The selected direct task now completes all 16 phases in 41.76 simulated seconds (about 29 minutes wall time while the fixture comparison ran concurrently). All 334,080 physics callbacks matched the requested steps. Its 1,253 video frames and 16 clips decoded successfully.

The fixture strategy now lands the hanging tail before laying down the held end. One trial reached regrasp but failed lift; the corrected-guide trial lost its suction seal during placement. Those earlier failures remain in the record. The final selected PGS fixture run succeeds: 66.789 simulated seconds, 534,312 matching physics callbacks, 2,004 decoded frames and 23 clips. It has 26 cable/shelf contact pairs, confirming actual passive support, followed by an independent mechanical regrasp. No unintended tool or arm/tool contact exceeds the declared gate.

## Strategy decision

| Measured result | Direct | Passive fixture |
|---|---:|---:|
| Geometric task audit | Pass | Pass |
| Simulated sequence duration | 41.760 s | 66.789 s |
| Recorded stages | 16 | 23 |
| Maximum sampled pre-insertion grip drift | 1.32 µm | 1.93 µm |
| Tip seating depth | 3.80105 mm | 3.80099 mm |

Use the direct sequence as the initial control baseline: it completes this nominal case with fewer handling stages. Retain the fixture branch for testing uncertain pickup poses, curled cables and reorientation. One trial per strategy cannot rank reliability; the timings reflect the chosen controller and are not predicted hardware cycle times. [Measured phase comparison](https://lakshya-asu.github.io/ffc-insertion-research/outputs/strategy-comparison/comparison.png) and [comparison data](https://lakshya-asu.github.io/ffc-insertion-research/outputs/strategy-comparison/comparison.json) are included.

## Reproduce and inspect

```bash
cd /home/flux/ffc-insertion-research
./scripts/docker_experiment.sh build
./scripts/docker_experiment.sh smoke --output /workspace/outputs/my-smoke
./scripts/docker_experiment.sh direct --output /workspace/outputs/my-direct
./scripts/docker_experiment.sh assembly --output /workspace/outputs/my-fixture
./scripts/docker_experiment.sh verify outputs/my-direct
uv run python scripts/audit_grip.py outputs/my-direct
uv run python scripts/audit_seating.py outputs/my-direct
```

Use a fresh output directory for each run. The selected PGS model uses 8 kHz physics, a 1 kHz controller, 255 position iterations and four velocity iterations, so full handling runs are slow. `scripts/run_pipeline.sh` builds, tests and runs the declared baseline suite while retaining failures. A failed strategy returns a failure status.

## Meaning of realism in this build

The arm uses official FR3 v2.1 geometry and joint frames. Cable gravity, bending, contacts, finite actuator drives, suction compliance/breakage, independent jaw motion and a passive fixture are simulated. Videos show the simulated states, not illustrative animation.

The 150 × 8 × 0.3 mm cable and 0.7 g mass are assumptions. Full-span static sag is 46.081 mm versus a 46.132 mm independent nominal reference; this qualifies the declared numerical shape check, not a real laminate. Dynamic/contact-force behavior is not calibrated. Earlier TGS contact estimates exhibited large numerical spikes. The selected PGS transfer peaked at 0.252 N per cable/tool contact pair; this is still an uncalibrated simulation estimate, not a hardware damage limit.

The tool is a mechanical concept with ideal guides and an assumed servo, not manufacturing CAD. The connector interior is an assumed FH12-sized clearance slot. Both successful insertions report zero cable/connector contact load: this demonstrates aligned geometric entry into the open slot, not terminal-spring engagement or force-controlled insertion against a measured connector resistance. The controller uses exact simulated state. There is no learned vision policy, automated latch closure, post-latch tool withdrawal, electrical continuity check or real-robot validation. A geometrically seated tip alone is not a completed electrical connection.

See [ISAAC.md](ISAAC.md), [contact/transport experiments](experiments/007-contact-and-transport.md), and the [physical calibration plan](CALIBRATION.md) for model definitions and the measurements needed next.
