# Commercial actuators with custom tool parts

This is a **CAD fit-review assembly**, not a released manufacturing design or a validated gripper. The user selected the approach: buy the motion/vacuum components and adapt them with printed parts. The present assembly preserves separate lower-finger deployment and clamping.

## What exists

- Authentic Actuonix PQ12 manufacturer STEP bodies and shafts, imported from the current official downloads page. The electrical variant is not identified by the family CAD; the design candidate is two PQ12-30-6-P units.
- A 47-component assembly with printed clevises, carriage, vacuum shoe and finger carrier; metal frame, guide-shaft, pin and thin-finger geometry; commercial bearing and suction-cup envelopes.
- Separate STEP and STL exports of the printed concept parts. These are for fit discussion only: fastener retention, print orientation, tolerances and structural qualification remain open.
- A 20-second Isaac RTX kinematic sequence. The assembly is moved by prescribed transforms, with zero physics advancement. Cable shape is prescribed too. It is not pickup/insertion success evidence.
- An independent 19-pose check of selected authored solids against each other and the desk. No checked interference was found in CAD revision 004. Exclusions are recorded in `audit.json` and must travel with this result.

## Mechanical sequence

The first arrangement would place the parked finger below the desk. Revision 004 fixes that by reserving an above-plane stow position:

| Pose | Deployment extension | Clamp extension | Tool lift |
|---|---:|---:|---:|
| Vacuum pickup, finger sideways and raised | 0 mm | 0 mm | 0 mm |
| Lift the cable | 0 mm | 0 mm | 35 mm |
| Lower the parked finger | 0 mm | 18 mm | 35 mm |
| Deploy underneath | 18 mm | 18 mm | 35 mm |
| Close to nominal ribbon body thickness | 18 mm | 8 mm | 35 mm |

Both actuator extensions stay within the published 20 mm stroke. These are geometric poses, not force-control commands. The nominal closed gap is 0.14 mm, taken from the current Pi cable body's assumed thickness. It does not include controlled pad compression. The animation uses a 200 mm Standard–Mini outline illustration with an 11.5 mm narrow region and 16 mm wide end, but does not qualify cable laminate mechanics or contact-face appearance.

The upper shoe remains fixed. The lower finger has an offset side support so its vertical riser does not run through the ribbon tail. Metal guides carry the finger's bending moment; the actuator rod is not used as a cantilever bearing. The long fixed frame is represented as metal stock. Printed parts attach and locate the mechanism; we do not assume that a thin printed edge or a long printed bracket is adequately stiff.

## Purchased components and limits

| Item | Design choice | Status |
|---|---|---|
| Deployment actuator | Actuonix PQ12-30-6-P | Actual manufacturer CAD imported; variant suitability unqualified |
| Clamp actuator | Same PQ12 family candidate | Requires external force feedback and series compliance; not direct gentle-force control |
| Suction interfaces | Two Schmalz SUF 3-size interfaces | Diameter/stem envelopes only; lip, hex fitting, seal and hose layout unfinished |
| Guides and bearings | 4 mm lateral shafts, 3 mm clamp shafts | Stock-size space claims; supplier, running fits and retaining features unselected |
| Force sensor | Orange space reservation | No sensor installed; no implemented compliant force path |
| Drive and acquisition | H-bridge + potentiometer acquisition + local controller | Not selected or implemented; no ROS actuator endpoint |
| Vacuum supply | Pump/ejector, valves, pressure sensing | Not selected; no tubing modeled |

The PQ12 datasheet lists ±0.1 mm positional repeatability, 0.25 mm backlash and a 20% maximum duty cycle. Its 30:1 version is rated for up to 18 N lifted load. These are not acceptable substitutes for measured clamp force or a continuous training duty cycle. The P version exposes an analog potentiometer and has no built-in controller. The generic tool's earlier assumed drive gains/force limit must not silently become the hardware model.

The next mechanical gate is the compliant force-sensing cartridge, with a real load path, overload behavior and selected electronics. A spring alone is not a force limit after it bottoms out. Loss of power does not guarantee release with a geared actuator. Neither behavior is modeled by this demonstration.

## Sources and geometry provenance

- [Actuonix product variant](https://www.actuonix.com/pq12-30-6-p)
- [Official current datasheet](https://www.actuonix.com/assets/images/datasheets/ActuonixPQ12Datasheet.pdf)
- [Official manufacturer STEP ZIP](https://www.actuonix.com/assets/images/datasheets/PQ12_NEWSHAFT_STP.zip)
- [Official downloads index](https://www.actuonix.com/datasheets)
- [Schmalz exact suction-cup data](https://www.schmalz.com/10.01.01.14292)

Manufacturer STEP SHA-256: `0a1eabc15536edc0bc455dbdd5f3971a070bc7a36ca059887ec54429d139d24b`.

The imported STEP contains seven shells: five body shells and two shaft shells, grouped into two manufacturer components. Pivot axes were inspected from its cylindrical faces. Closed pivot spacing is 42.013 mm in this model. Original surfaces and the full STEP assembly remain local in ignored `outputs/`; the public assembly STEP substitutes labelled actuator bounding envelopes. The public printed-parts STEP/STL files contain our own geometry. Do not mistake the envelope download for detailed vendor CAD.

## Reproduction

Use a separate CPU environment with `cadquery==2.6.1`, `cadquery-ocp==7.8.1.1.post1` and Python 3.12. Download and unpack the official PQ12 STEP separately. Our first command-line download returned HTTP 403; a normal Playwright browser request succeeded. Do not substitute a different supplier model without inspecting the component partition.

```bash
python design/hybrid-tool/build.py --source PQ12_NEWSHAFT.stp --output outputs/hybrid-tool-cad-NEW
python design/hybrid-tool/audit.py outputs/hybrid-tool-cad-NEW
```

Pause the live preview and wait for it to stop before the GPU render. Use the existing Isaac container:

```bash
docker compose run --rm experiment design/hybrid-tool/render_isaac.py \
  --cad /workspace/outputs/hybrid-tool-cad-NEW \
  --output /workspace/outputs/hybrid-tool-render-NEW
```

Check `review.json`, the encoded video and images, not only process exit. Restore the live preview afterwards. No old Pi-cell controller or actuator configuration is changed by this work.

## What remains before a build

The force cartridge, exact guides, screw retention, threaded inserts, vacuum plumbing, print tolerances, FR3 adapter/inertia and camera clearance remain unresolved. This larger prototype has not inherited the previous tool's clearance results. The sampled solid check excludes supplier actuator shells, same-motion-group pairs, the sensor reservation, cable and absent fasteners. It is not a swept-volume proof. Housing interference, tube bends and tool stiffness still need checking.

For Isaac dynamics, add actual travel/backlash/deadband and duty constraints, compliant contact pads, measured-signal emulation and a declared suction/leakage model. Runtime decisions must use camera, jaw position, force and pressure observations. Simulator geometry may support collision modeling and offline scoring, not hidden-state grasp decisions.

## Mounted dynamics follow-up

[Experiment 030](../../experiments/030-mounted-tool-motion.md) mounts this CAD on the FR3 with a provisional adapter and two driven prismatic joints. The empty-tool physics sequence and cancellation trials are separate from the zero-time kinematic review above. The force cartridge, vacuum model and cable grasp remain unqualified. The adapter in the dynamics scene is a structural envelope, not a manufacturing release; the original standalone STEP downloads do not include it.
