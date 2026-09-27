# E003: handling sequence and cable material checks

Written before simulator execution. Seed 260927. Simulation only.

## Handling hypothesis

The offset tool with upward jaw parking can acquire the end of a flat ribbon, carry it to a passive shelf, release it, regrasp by frictional pinch, and present the tip to a connector without robot/environment contact.

Use the segmented model for initial controller debugging. A proximity/orientation-gated breakable fixed joint models suction acquisition. It is released at the fixture. No attachment substitutes for the subsequent pinch grasp. A passed sequence would demonstrate the simulated mechanism under assumed contact parameters, not real suction reliability or hardware assembly yield.

Record all phases, joint state, cable state, rigid contact impulses, video, and gate failures. Reject tool tracking error above 1.5 mm, unsuccessful lift, cable loss from the shelf, and excessive connector contact. The 0.5 N connector and 0.1 N unintended robot/environment contact thresholds are experiment abort settings, not validated component safety limits. Forces are sums of contact impulse magnitudes divided by timestep, not a calibrated six-axis wrist measurement.

Geometric seating requires the tip to reach within 1 mm of nominal insertion depth, lateral error below 0.2 mm and height error below 0.1 mm. Latch closure, electrical continuity, conductor face compatibility and damage must be reported separately. A seated tip alone is not full task success.

## Material hypothesis

The native surface-deformable ribbon should sag under gravity and converge as mesh and timestep are refined. Its short-span cantilever response should be consistent in scale with a homogeneous beam estimate using the same mass, width, thickness and Young's modulus.

Cantilever: 50 mm total length, first two vertex rows anchored, mass scaled from the 150 mm ribbon. Predict free-end sag with uniform-load Euler–Bernoulli beam theory, q L^4/(8 EI). Report the ratio rather than silently changing material constants to fit. A finite-width surface and its Poisson response need not exactly match a one-dimensional beam.

Run the nominal material first, then mesh/timestep refinement and stiffness sensitivity if the native model loads. Material parameters remain assumptions. The solver supports only an effective homogeneous surface; copper traces, reinforced ends, hysteresis, damage and true static friction are unresolved in this mode.

## Results

Pending execution. No successful pickup, regrasp, insertion, or material validation has yet been measured.
