# 038 — First contact-driven motion with the full cable

This pilot closes the compact fingers on a presented insulated section of the assumed 200 mm cable, lifts its end 10 mm and holds it. The tail remains on a support. It is local gripper motion, not an FR3 pickup or insertion demonstration. Camera/ROS integration and whole-tool clearance remain open.

## Results

| Case | Controller result | Independent observation |
|---|---|---|
| `flex-pinch-001` | Bilateral contact, lift and hold complete at 3.000125 s | Tool lift 9.992 mm; original cable material point at the grasp rises 9.973 mm |
| `flex-empty-001` | `empty_or_too_thin` at 0.465125 s | Lift command stays zero |
| `flex-dropout-001` | `contact_lost` after the load signal is zeroed during lifting | Lift reference freezes at 2 mm; additional measured lift is about 0.00036 mm over the following 0.25 s |

These are individual pilot runs, not a reliability estimate. The dropout is an injected sensor failure; it is not a physically slipping cable. The positive completion state remains `contact_hold_complete`, not verified grasp success. Its final pad readings are about 0.151 and 0.148 N under the idealized sensor/drive model.

The offline lift measurement interpolates segment-center positions at the original 10.5 mm material coordinate. It confirms that cable material moved with the tool in this run. It does not establish camera observability, damage absence, full-cable clearance or insertion readiness. The task controller never receives these positions.

## Cable and contact model

`ffc.profile_cable` creates a free articulation with 100 two-millimetre sections. Width follows the pinned revision-two profile; both ends have the assumed six-millimetre reinforcement. The 0.14 mm body and 0.30 mm terminal thicknesses, equivalent 3 GPa modulus and 1800 kg/m³ density remain declared assumptions. Local mass and inertia follow each section. Adjacent half-section bending compliances are added in series. No joint attaches the cable to the tool or ground.

The 100-section mass is approximately 0.65963 g. Tests compare total mass against the exact integral of the declared width/thickness profile, check the finer 200-section result, and verify the series-compliance behavior. This checks construction arithmetic, not identified real material properties. Width is approximated by midpoint boxes; copper strips are visual markings with an unverified face convention.

This pilot uses PGS, 0.125 ms steps, 255 position and eight velocity iterations. Angular viscosity uses the 0.00001 multiplier from the bending investigation; artificial body drag remains 20/s. Twist/in-plane stiffness, friction and dynamic damping are unqualified assumptions. The full-profile, free-articulation contact problem differs from experiment 037's clamped homogeneous strips, and its timestep is four times larger. The short-strip pass is not inherited as contact qualification. A mesh/timestep comparison and contact-reference checks remain necessary.

Only the tool's pad boxes collide. The supplier/finger CAD is visual, and a prismatic bench axis substitutes for the arm. The pad footprint starts 9–12 mm behind the tip, outside the assumed 4 mm exposed-contact region on either face. The presented orientation is a contact-bench layout; it does not establish a usable insertion pose or a visible leading edge. The next loaded-tool review must include the housing, fingers, mount and camera.

## Feedback and motion

The existing `PinchSkill` receives timestamped tool displacement as an encoder equivalent and two pad-load readings. The adapter averages ten milliseconds of pad contact impulse magnitude divided by timestep, discarding the counterpart identity. These are idealized aggregate loads, not calibrated tactile normal/shear arrays.

The controller now updates at a proposed 200 Hz, independently of the physics step. That is a local interface experiment, not verified actuator bus or ROS timing. Finite-force prismatic drives execute the targets. The fingers begin at a deliberately presented 1 mm opening so this pilot isolates contact and retention. There is no perception-guided approach.

The inherited bench thresholds and drives remain assumptions: bilateral load above 0.08 N for 50 ms, a 0.8 N jaw-drive cap, and a bounded 5 mm/s lift. They are not damage limits or an identified PGEA motor model. The tested commercial gripper remains one candidate among the retained tool options.

The actor's inputs contain no cable pose, attachment status, reference curve or simulator segmentation. Cable positions/quaternions are written to a separate offline trace. Rendered cameras supply review footage only. The experiment records sensor observations, commands, cable trace, section properties, a final USD state and actual RTX video. Source files are copied into each fresh run directory.

## Reproduce

```bash
docker compose run --rm experiment scripts/isaac_flexible_pinch.py \
  --case cable --dt 0.000125 \
  --output /workspace/outputs/flex-pinch-NEW
```

Other cases are `empty` and `load_dropout`. The runner also accepts smaller timesteps and 200 sections for subsequent numerical comparisons; those comparisons have not yet been run. Use a fresh directory and keep one writer on the human-view feed.

## Next motion milestone

Mount the moving candidate tool on the FR3 and qualify its loaded clearance, including the cable. Verify leading-edge visibility in the intended observation cameras, then connect the bounded motion and tool observations through the existing ROS interfaces. Check the full-cable contact model at a finer timestep before treating its behavior as qualified. The first integrated task remains a sensor-verified flexible-cable pickup; insertion follows from stable retention and visual alignment.
