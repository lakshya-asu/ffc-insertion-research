# E005 — numerical compliance and handling validation

The early handling trials lifted and regrasped the cable, but the ribbon looked too flexible. A cantilever benchmark confirmed that the original 1/240 s segmented solver configuration was unsuitable for interpreting material behavior. Its gravity sag was approximately 25 times the static prediction for the authored hinge chain. Those early videos remain development evidence, not a validated material simulation.

## Segmented benchmark

A homogeneous 50 mm cable is made from ten 5 mm segments. The first segment is fixed, leaving nine free segments. End stiffening is disabled for this benchmark. Mass is scaled with length from 0.7 g per 150 mm. The nominal beam has width 8 mm, thickness 0.3 mm and E = 3 GPa, giving EI = 5.4e-5 N m². Its distributed gravity load is approximately 0.04578 N/m.

For a continuum cantilever with free length L = 45 mm, tip deflection is q L⁴/(8 EI) = 0.43455 mm. The actual authored rigid chain has a full rotational spring at the first free hinge. Its small-angle static prediction is therefore different:

`delta_chain = q * ds^3 / (2 * k_rad) * sum(j^3, j=1..N)`

Here N = 9, ds = 5 mm, and k_rad = EI/ds. The reference is **0.53648 mm**. The discrete boundary contributes a factor `(N+1)^2/N^2 = 1.23457` relative to the continuum expression. This formula is derived from moment balance for the authored chain; it is not fitted to measured simulation output.

| Physics timestep | Position iterations | Simulated sag | Error relative to discrete prediction |
|---|---:|---:|---:|
| 4.1667 ms | 32 | 13.361 mm | +2,391% |
| 0.25 ms | 128 | 0.83577 mm | +55.8% |
| 0.125 ms | 255 | 0.61686 mm | +15.0% |
| 0.0625 ms | 255 | 0.56130 mm | +4.6% |

The material spring stiffness was unchanged across these rows. A separate hypothesis test multiplied angular gains by 57.2958; it did not repair coarse-step solver compliance and is excluded from the nominal-material comparison. USD angular stiffness is authored in N m/degree and converted internally; the original units were correct.

The 8 kHz profile is being used for the next full handling trial, with its approximately 15% residual numerical error explicitly retained. The 16 kHz benchmark is a finer reference, not a demonstrated full-episode result. These 0.5 s gravity tests do not establish convergence for large deflection, frictional contact or the full 150 mm cable. Longer spans and contact-rich states need additional comparisons.

Results and frozen source are in `outputs/e006-*`. Rebuild the independent comparison using `uv run python scripts/compare_segment_benchmarks.py`. See [the plot](../outputs/benchmarks/segment-convergence.png) and [machine-readable comparison](../outputs/benchmarks/segment-convergence.json). Early benchmark results call a finite plausible deflection PASS separately from their beam-agreement flag; inspect the metrics rather than interpreting PASS as calibrated mechanics.

## Native surface FEM

The surface solver requires GPU dynamics enabled after World creation, because World initialization otherwise resets the USD flag. Once enabled, the finite-thickness shell settles under gravity. Numerical sweeps also show strong timestep, iteration and mesh sensitivity:

| Mesh | Timestep | Position iterations | Sag / continuum reference |
|---|---:|---:|---:|
| 30 × 4 | 0.5 ms | 128 | 1.438 |
| 30 × 4 | 0.25 ms | 128 | 1.323 |
| 60 × 4 | 0.25 ms | 128 | 1.906 |
| 60 × 4 | 0.25 ms | 255 | 1.324 |

A 512-iteration request exceeded the schema's 1–255 range and is excluded. It produced the same measurement as 255, consistent with clamping, but that inference is not used as a valid configuration. The installed schema is now enforced by the model builder.

See [native-shell comparison](../outputs/benchmarks/shell-convergence.png). Full native-shell handling has not been implemented; its current role is a mechanics comparison.

## Handling findings before refinement

Both direct and fixture strategies completed their transfer stages in E005 handling trials. The direct strategy retained the cable after both vacuum patches were removed. The fixture strategy placed the cable, removed suction, cleared the fixture, regrasped and lifted it through contact alone.

Both failed the final seating gate. Direct trial `e005-direct02` measured only 0.155 mm insertion; the gripper moved further while the cable slipped. The fixture trial `e005-fixture02` left the tip 3.322 mm short of the mouth. The next tool configuration increases nominal pinch preload from approximately 0.09 N to 0.36 N using the same finite 600 N/m drive, with a 2 N force cap. Preload is a drive-deflection estimate and must be checked against actual contact records.

These changes address observed failures. They do not establish insertion success until a new full episode passes the unchanged seating and contact gates.
