# E006 — full-span mechanics and handoff debugging

This extends the short cantilever checks to the complete 150 mm ribbon. It also isolates the failure observed during the vacuum-to-pinch handoff. Runs E007–E026 preserve each tested configuration and its source snapshot; a runtime PASS alone does not qualify a material model or an insertion.

## Independent reference and full-span behavior

`ffc.cantilever_reference.hinge_chain_reference` minimizes the exact gravitational-plus-spring potential of a planar chain with ideal translational joints. Its one-free-link result is tested against an independent scalar torque-balance root. For the 30-segment, 150 mm homogeneous ribbon with its first segment fixed, the nonlinear reference tip sag is 46.1317 mm. The continuum small-deflection reference is 46.8451 mm; the discrete linear reference is 50.1315 mm.

A freely released full-length cable continues oscillating after two seconds. Its instantaneous sag must not be compared to a static reference as though it had settled. Increasing joint viscosity by 100× caused severe numerical drive/solver sensitivity, so that approach was rejected. PGS, reduced-coordinate articulation, zero velocity iterations, external-force updates, and unit-consistent gram scaling were also investigated. None of those tests alone established a validated full-length dynamic model. The three-segment reference did agree within 0.7%, helping distinguish a long-chain numerical issue from a simple unit error.

NVIDIA documents [D6/TGS drive limitations and nonzero reported velocity at steady state](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.0/dev_guide/guides/current_limitations.html). These informed the diagnostic sweeps. They are not a justification for silently accepting an incorrect model.

## Static-shape qualification

For the equilibrium check only, a 20 s⁻¹ body velocity damping term removes transient energy while preserving the zero-velocity equilibrium. This extra drag is explicitly rejected by the runner outside cantilever benchmarks. The original joint stiffness and viscosity are retained.

Run `e022-static-shape-audit02` uses the main 8 kHz, 255-iteration segmented profile. It measured 51.0736 mm tip sag, 10.7% above the nonlinear reference. The static-shape protocol requires agreement within 20%, a quiet final coarse sampling window, and a maximum surface-point excursion below 1% of cable thickness (3 μm) during a dense 25 ms window. The latter bound includes translation and rotation of every segment and is sampled at every physics step.

The measured dense-window surface excursion bound was **0.0215 μm**, and the static-shape check passed. A stricter instantaneous velocity criterion did not pass: solver-reported angular velocities disagree with finite-difference pose motion, and tiny orientation jitter is amplified by differentiation. Both velocity estimates and the stricter failed flag remain in the results. The awake-body repeat `e027-awake-static-shape` produced the same measurements with sleeping and stabilization disabled. This is a qualified static-shape approximation, not validation of dynamic forces, contact damping or real cable material.

Reproduce in a fresh directory:

```bash
./scripts/docker_experiment.sh cantilever \
  --config /workspace/config/equilibrium-body-drag8k.json \
  --cantilever-length .15 --steps 16000 \
  --output /workspace/outputs/my-full-span-check
```

`e022-static-shape-audit` was a failed instrumentation attempt due to an uninitialized audit field; it is retained. The corrected run is `e022-static-shape-audit02`. Its full video, frozen source hashes, 60 decoded frames and stage clip passed artifact verification.

## Handoff findings

The refined direct trial `e007-direct-refined` lifted the cable but dropped it after vacuum release. The fixture trial `e007-fixture-refined` exceeded the suction displacement limit while placing the cable. These failures supersede any implication that the earlier coarse-step handling videos establish a robust mechanism.

The vacuum joints initially suppressed cable/upper-shoe collision. That suppression was removed because the mechanical pinch needs that contact. A separate test retained physical contact while disabling the vacuum joints, but still dropped the cable. The current diagnostic zeroes all six drive stiffnesses, dampings and force limits while keeping each free D6 joint in place, to isolate constraint-topology changes from actual grip behavior. A vented joint has no locked axes and no force-producing drive.

Position targets, velocity targets, gains and applied feedforward efforts are now recorded alongside actual joint positions. A pure mechanical-pinch diagnostic starts with the cable already between the jaws, uses no suction, and is explicitly separate from desk pickup. The first invocation accidentally selected tool exercise and was stopped as ABORTED; `e026-pure-pinch02` is the corrected invocation. `e026-pure-pinch02` passed without suction. Its jaw held near −6.2 mm against a −6.8 mm target, demonstrating finite-force mechanical retention.

The drive telemetry then isolated the actual handoff bug: phase initialization restarted a constant clamp target from the measured, contact-loaded jaw position. That removed the approximately 0.384 N preload precisely when vacuum was released, before ramping it back. `retain_unchanged_targets` now preserves unchanged tool commands across skill boundaries. Two regression tests check constant preload over all interpolation fractions and smooth motion when the command actually changes. E028 contains the replacement transfer and full handling trials. E025 trials were deliberately superseded and their unfinished video files are marked partial.

## Controller corrections under test

The new full trials start from a collision-gated ready pose above the desk; the cable remains ungrasped. Before connector alignment and insertion, they correct the tool target from the actual simulated cable-tip pose. This is privileged-state feedback, not vision. The fixture variant additionally lands the free tail on the rear shelf, then moves forward and down to lay the cable instead of compressing a hanging ribbon vertically onto the shelf.

Geometric seating remains separate from latch closure, tool release, electrical continuity, damage inspection, randomized-start success rates and hardware transfer.


## Guide-rod interference and shutdown audit

E028 kept the clamp target at −6.8 mm through venting, but transfer still failed. Its contact log identifies an unintended guide-rod/upper-shoe contact peaking at approximately 6,040 N, despite a 2 N actuator limit. The old rod inner face was tangent to the shoe side face. These impulses invalidate that handling result as a physical tool demonstration; preserving controller preload was necessary but insufficient. The rod has been moved outward to provide 3 mm geometric side clearance and connected to the jaw with a lug. A new 0.1 N gate rejects unintended tool/tool contact. E030 reruns this corrected geometry. No contact filter was added to hide this collision.

The first Docker interruption test required forced termination. The revised signal handler records a flag and raises an interruption in the Python simulation loop, outside native callbacks. E030 stop-check02 preserved an ABORTED result, 36 decoded video frames, two clips and a matching 1,382-step clock audit. Its source hashes and media were verified. Explicit Kit privacy flags also prevent the anonymous telemetry transmitter from launching; the environment consent flag alone did not.


## Corrected transfer result

E030 transfer-clearance passed acquisition, lift, lower-jaw deployment, pinch and vacuum release. The cable remained mechanically retained through the final hold. The 11.647 s run has 93,176 matching physics callbacks, 350 decoded frames and seven stage clips; source hashes and media decode checks passed. No unintended arm/tool or tool/tool contacts exceeded the 0.1 N gate.

This is a retention result, not a calibrated force result. The all-step peak cable/tool contact proxy reached 6.915 N during the post-vent hold, while the 100 Hz sampled late-hold maximum was 1.084 N. These impulse/timestep peaks are numerically noisy and exceed the nominal quasi-static clamp preload; they must not be interpreted as measured real fingertip loads or proof against cable damage. Full transport and connector seating are tested separately.
