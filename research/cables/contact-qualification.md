# From bending curves to a cable we can pick up

The next skill is a supported flexible-cable pickup in simulation. Before connecting it to the FR3, isolate the mechanics that can make a convincing-looking pickup misleading. These are proposed experiments, not completed results. The seven-skill tracker remains the task-level record.

## What experiment 037 contributes

It tests out-of-plane static bending of homogeneous 24–40 mm strips against an independent equilibrium solve. Both share the declared elastic hinge-chain model. The short-strip solver settings are a starting point for the next benches, not a transferable qualification of the full cable, damping or contact. The numerical setting of 32,000 physics steps per simulated second also needs a cost study before large-scale training. It is unrelated to the camera or controller update rate.

## 1. Build and audit the full profile

Use the pinned 200 mm Standard–Mini revision-two profile: 11.5 mm narrow body, transition to the 16 mm end, and terminal reinforcements on both ends. Published dimensions and assumed dimensions must keep their source labels. Use the same geometric profile for rendering and collision; do not give the collision model a thick hidden body to make contact easier.

Represent each segment's mass, inertia and bending rigidity from its local width and thickness. For adjacent unequal sections, combine their half-segment bending compliances in series. Make the reference and physics models use the same neutral-axis and boundary conventions. Preserve the assumed contact and reinforcement lengths; they are not manufacturer tolerances. Keep exposed conductor regions excluded from proposed pad footprints on both faces.

Check total mass, center of mass, local stiffness changes and joint-frame continuity before stepping physics. Compare a supported span and a held end at two spatial resolutions and at a halved timestep. Use static settling aids only in static comparisons. A release or lift experiment must declare its dynamic damping separately.

## 2. Qualify contact without a robot

| Bench | Independent offline check | Failure that it should reveal |
|---|---|---|
| Flat cable on desk | Lowest body surface, total contact support versus weight, residual velocity | Floating on contact offsets, penetration or persistent jitter |
| Cable over a support edge | Centerline and local curvature; mesh/timestep comparison | Joint gaps, excessive sag or false stiffness from contact |
| Insulated coupon between pads | Pad travel/load and known coupon thickness | Collider mismatch or closure through the sample |
| Controlled tangential pull | Applied load, pad normal load and measured slip | Incorrect friction combination or artificial sticking |
| Full cable with a held end | Shape, retention and load during a bounded lift | Numerical attachment, slipping or unstable articulation |

Treat desk/cable and pad/cable friction as separate declared assumptions. Sweep them independently. Begin with simple contact references whose support force or slip bound can be calculated without the simulator, then introduce the cable's bending and changing contact area. State friction-combination rules and use the resulting effective coefficient in the reference calculation.

Contact offset is not physical thickness or penetration tolerance. Log both contact distance and geometric overlap. Set acceptance budgets before evaluating a run, using cable thickness and the connector's eventual clearance budget. Numerical convergence criteria are engineering choices, not measured damage limits. Preserve unsuccessful cases and change the model version when a failed validation informs a revision.

## 3. Add a bounded grasp and observe it

Begin with a deliberately presented insulated section so the first experiment tests retention rather than mixing in pose-estimation errors. Reuse the existing rigid-coupon pinch controller's observation boundary: pad loads and actuator encoders may guide closure, while the offline evaluator records true cable shape and slip. Recheck the load model and thresholds for the flexible sample; a rigid coupon does not qualify them.

Next add the desk pickup and compare the direct tool maneuver with the passive presentation fixture. Keep tool options open. A suction variant needs a declared pressure, seal/leak and holding-force model; proximity or a simulator attachment flag cannot stand in for a seal measurement. Cable retention must come from modeled forces and contact, not moving the cable to the tool pose.

Runtime grasp checks should combine pad/pressure observations with camera evidence of retention and an exposed leading edge. Inject empty grasps, low friction, sensor dropout and delayed images. A failed check should stop the lift or return to a bounded recovery state. Ground-truth pose is permitted only in a separate evaluator.

## 4. Connect the successful bench to the cell

Only after the local bench passes, add the mounted FR3, camera viewpoints and ROS2 timing. The loaded tool needs a fresh clearance review. Track image acquisition time, robot joints, pad/pressure samples and controller decisions on a consistent clock; check freshness after each bounded motion. Physics stepping faster than the cameras is expected, but the controller must not silently receive perfect high-rate object state between frames.

The first integrated deliverable is a recorded pickup with sensor evidence of stable retention and an inspectable cable end. It is step 2 of the seven skills. Inspection/orientation, visual alignment, connector contact, seating/latching and recovery follow in that order. Dataset collection can then capture successful and failed trials from the qualified task configuration; training does not substitute for those checks.

## What to publish from each bench

Archive the model/profile hash, solver and timestep, material/contact assumptions, initial conditions, commands, sensor observations, independent scoring and a rendered recording. Include both successful and failed cases. Keep reference geometry and offline labels out of the policy observations. Report computational cost as well as physical agreement so the eventual learning environment has a credible throughput estimate.
