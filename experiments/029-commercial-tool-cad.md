# Commercial-actuator tool CAD and kinematic review

The user chose commercial motion/vacuum components with custom printed tooling, then requested the CAD assembly and a demonstration of its operation. The existing two-axis concept is retained instead of pretending a parallel gripper is a drop-in replacement.

## Delivered artifacts

- CAD revision `outputs/hybrid-tool-cad-004`: 47 components, including 16 custom printed-part concepts. All shapes pass BREP validity; each printed part is one solid after fusing clevis pieces.
- Manufacturer PQ12 STEP surfaces form both actuator bodies and shafts. The candidate electrical variant is PQ12-30-6-P. Guides, bearings and cups remain approximate envelopes; the force sensor is only a space reservation.
- The local full assembly is `outputs/hybrid-tool-cad-004/assembly.step`. The public STEP substitutes actuator envelopes and links to the manufacturer's original download. The custom printed STEP and STL archive are public fit-review artifacts.
- `outputs/hybrid-tool-render-003`: 240 actual Isaac RTX frames at 12 fps, producing a 20-second kinematic video. It includes an overview, close-up and ten sequence stages. Assembled and separated-subassembly renders are also provided.
- [Review page](https://lakshya-asu.github.io/ffc-insertion-research/hardware/custom-gripper.html) and [parametric source](../design/hybrid-tool/README.md).

## A real design issue found

The initial design left guide/finger geometry below the desk at pickup. Applying the sampled review to revision 001 records four floor-intrusion entries. That earlier assembly and report remain local. The revised design raises the clamp hardware, adds a raised sideways stow position and offsets the finger riser around the cable tail.

At pickup, both actuators are retracted. The tool lifts 35 mm in the illustration; clamp extension then moves to 18 mm while parked, deployment extends 18 mm, and the clamp retracts to 8 mm extension. Reversal opens first, withdraws sideways, raises the parked finger, then lowers the tool. Both actuator positions remain within their 20 mm travel. These distances demonstrate layout, not qualified grasp settings.

## Checks and limits

The independent CAD audit sampled 19 poses. It found no desk intrusion or interpenetration above 0.01 mm³ among the included solid pairs in revision 004. It excludes actuator shells, same-motion-group pairs, the force-sensor reservation, cable and unmodeled fasteners. There is no continuous collision guarantee or full-cell clearance qualification.

Every rendering step asserts zero simulation-time advancement. Actuator and cable movement is prescribed. No rigid-body joint-drive test, suction-flow model, contact force, force-feedback loop, grip retention or insertion success is established by this video. The persistent video banner and page state this explicitly.

The PQ12 backlash, repeatability and duty cycle prevent treating its position command as gentle clamp force or continuous-duty actuation. The compliant force cartridge, electronics, exact guides, fastener retention, plumbing, FR3 interface and tool stiffness remain design gates. The orange block does not represent an installed sensor.

The render records its CAD-manifest hash; publication rejects mismatched inputs. Local browser checks cover desktop, mobile and dark mode, phase selection, assembly-view switching and video metadata. This static design page introduces no actuator endpoint. The live sensor-only Pi preview is restored after rendering.
