# An assumed world we can build and learn in

We can proceed with a simulation proof of concept before identifying the exact physical cable. The research and declared assumptions define an initial reference world. We then build a computationally practical simulator that approximates its responses and train policies using deployable sensor observations inside that world.

The claim is success within a specified model and parameter envelope. Later physical samples can update that model. Their absence does not prevent the initial proof of concept.

## What fitting means here

Keep two sets of parameters separate. The reference model contains the assumed material properties and constitutive laws. The fast simulator has effective numerical parameters fitted to reproduce reference responses at a fixed mesh, timestep and solver configuration. An effective joint gain need not equal a literal material modulus; it must be recorded as a fitted simulation parameter.

This qualifies the earlier warning against changing stiffness to hide numerical error. A single unexplained gain adjustment is not material identification. Explicit surrogate calibration, evaluated on independent cases and versioned with its numerical settings, is a valid development path. If the chosen representation cannot fit the response family, change the representation rather than silently changing the target world.

## Implemented first reference set

`config/cables/world-model-v1.json` pins the source-aware cable profile. `scripts/build_world_model_targets.py` generates static out-of-plane targets from nonlinear elastic hinge-chain energy minimization. It freezes its configuration, source and hashes in a fresh output directory.

- Fit spans: 24 and 32 mm; validation spans: 28 and 40 mm.
- Five distributed load multipliers: 0.25, 0.5, 1, 1.5 and 2.
- Three equivalent modulus multipliers: 0.25, 1 and 4.
- Fixed 2 mm segments, with the first whole segment clamped.
- Total: 30 fitting and 30 validation cases. These are synthetic reference responses, not observed physical measurements or policy-training episodes.

The load multiplier changes static distributed loading. It is not evidence for dynamic mass or damping. The initial targets omit stiffeners, contact and twist so the first fit has an interpretable scope. Run `outputs/world-model-targets-001` generated all 60 responses successfully. Experiment [037](../../experiments/037-assumed-world-fit.md) fits the Isaac numerical response and records the independent evaluation. It also documents a small-angle scoring correction and complete repeat runs. The reference and simulator use the same discrete elastic constitutive model but separate solvers; this tests numerical agreement, not continuum FEA or real material identification.

## Building the rest of the world

Start with bending force/displacement across multiple spans and loads. Extend the reference to the full width profile and stiffeners, directional twist and in-plane deformation. Add release/oscillation responses for damping, contact and slip curves for the desk and fingertips, and pressure/holding behavior for suction. Connector entry, buckling, latch motion and damage require their own declared models and validation cases.

Use a normalized multi-response fitting objective so one large displacement does not dominate small contact or orientation errors. Freeze the fitting procedure before evaluating validation cases. If validation informs another model revision, preserve that result and introduce a new untouched evaluation set before making final performance claims.

The resulting world can support a sensor-feedback controller, demonstration collection, imitation learning and later constrained RL. The policy does not receive cable ground-truth geometry. Simulator geometry remains available for offline supervision and scoring only. Task mechanics and sensor-boundary tests still need to pass; physical material identification is not an admission requirement for a simulation-only result.

No dataset release is granted by merely naming the reference world. Existing failed runs remain useful calibration evidence. The next executable milestone is fitting and independently checking the fast bending model, followed by the pickup contact model.
