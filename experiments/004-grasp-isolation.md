# E004 — isolate suction acquisition and lift

This diagnostic initializes FR3 at the solved pickup pose and the ribbon at rest on the desk. It tests acquisition and lift only; it does not test the preceding approach or insertion.

The isolated test was introduced after E003 completed the approach but lost the cable during lift. The original rigid attachment formulation placed the bending load into the rotational constraints and exceeded their assumed break torque. Two rigid patches also broke at simulation step 561. The event log identifies both broken joints.

The revised surrogate uses two 3 mm vacuum patches, 5 mm apart. Each patch assumes 30 kPa pressure differential, giving an ideal holding force of 0.212 N. Translational drive stiffness is 1,000 N/m normal to the cup and 300 N/m laterally; damping is 0.2 N·s/m. Angular compliance is 1e-6 N·m/degree, with a 1e-4 N·m torque limit. A 1 mm separation limit terminates the run. These are explicit design assumptions, not measured cup properties or a flow/leakage model.

The compliant pair can carry the ribbon's bending moment through a force couple instead of imposing two perfectly rigid rotational constraints. Both attachments are removed when vacuum is vented; subsequent pinch retention must come from contact physics.

`outputs/e004-grasp-diagnostic02` passed the implemented lift gate and the physics-clock audit. Its 192-frame MP4 and three stage clips decode successfully. The recorded ribbon hangs and bends under gravity. No electrical, damage, seal reliability, or hardware validation is implied.

Reproduce the diagnostic in a fresh output directory:

```bash
docker compose run --rm experiment scripts/isaac_workcell.py \
  --cable-model segments --grasp-benchmark \
  --config /workspace/config/grasp-benchmark.json \
  --output /workspace/outputs/grasp-repeat
```

Earlier E002/E003 runs without `timestep_audit` in their result metadata are development diagnostics. Their rendered events remain useful, but their loop-based elapsed times are superseded by the audited runner.
