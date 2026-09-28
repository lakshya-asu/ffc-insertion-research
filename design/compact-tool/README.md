# Compact commercial tool evaluation

We are evaluating a DH Robotics PGEA-2-10 instead of continuing the large two-PQ12 mechanism. The existing hybrid CAD and its motion evidence remain unchanged for comparison. This is a candidate study, not a purchasing decision or a qualified replacement.

## Verified starting point

The [manufacturer catalogue](https://en.dh-robotics.com/wp-content/uploads/2025/06/DH_PGEAPGIA-catalog_V255.pdf), PDF page 6 (printed pages 09/10), specifies a 150 g gripper, 10 mm total stroke and 0.8–2 N per jaw. Side-exit dimensions are 89 × 30 × 18 mm; bottom-exit dimensions are 94 × 30 × 18 mm. The small version uses an external driver (78 × 52.4 × 27.2 mm). Do not inherit integrated-controller assumptions from the larger PGEA variants.

A machine-readable candidate profile lives in [config/pgea-2-10.json](../../config/pgea-2-10.json). It is not a device driver or an executable robot configuration.

## Proposed arrangement

- One commercial pinch gripper with replaceable, thin custom fingertips that close across cable thickness.
- One fixed vacuum pickup nozzle beside the gripper, with pressure feedback. Its position must permit pickup without either finger reaching the desk.
- A passive presentation fixture that supports the cable while leaving the chosen grasp region accessible from above and below.
- A compact FR3 adapter with strain relief. Keep the external controller off the wrist if the specified cable length and routing permit it.

The proposed sequence is vacuum pickup → deposit on fixture → release suction → retract/reorient wrist → pinch the presented end → inspect → approach. The handoff is not automatic free-space transfer: the fixture supports the cable while the robot changes which tool surface it uses. Direct pickup remains a comparison branch.

## Checks before geometry is accepted

1. Verify the exact supplier jaw datum, stroke convention, mounting holes, screw engagement and cable keep-out using the detailed drawing and CAD. The catalogue's 1–11 mm indicated opening requires inspection before treating it as a working pad gap.
2. Define fingertip offsets, pad thickness and contact patch. The ribbon's 0.14 mm body and 0.3 mm header are current simulation assumptions, not specimen measurements. A commanded jaw position cannot substitute for measured contact force.
3. Compare the 11.5 mm mini end and 16 mm standard end. A 10 mm stroke does not rule out thickness pinching, but neither does it establish widthwise grasp feasibility.
4. Check camera visibility, lower-finger access, robot/PCB/camera clearance and nozzle clearance throughout pickup, fixture handoff and insertion approach.
5. Define the seal/leak, slip and fingertip-force observation models before declaring a simulated grasp. Use manufacturer force/speed curves as bounded initial models, not cable damage limits.
6. Verify the exact controller manual and register map before writing a ROS 2 hardware driver. Report command acknowledgement, measured gap, ready/fault state and timestamp separately; do not infer successful retention from a command completing.

## Source access

Direct downloads from the manufacturer's site returned HTTP 403 on this lab host. The catalogue remains readable through the research browser. The [TechShare distribution page](https://techshare.co.jp/faq/dhrobotics/dh-download-portal.html) links a public PGEA CAD folder with both side- and bottom-exit supplier drawings. Original supplier files are retained locally in ignored `outputs/`, with provenance and hashes in the import review. Do not substitute old PGE-series CAD or fabricate detailed supplier geometry.

## Lab session

The user confirms access to I2RT YAM and UFACTORY Lite 6. Work remains simulation-only; see the [arm comparison protocol](robot-candidates.md). Neither arm is ruled out or qualified for insertion.

Both supplier variants are now imported. The O-B drawing/model has a bottom/rear cable exit and 94 mm length; O-S has a side exit and 89 mm length. The English catalogue suffix table conflicts with those files, so the ordering code remains unresolved. The original local `pgea-2-10-side.STEP` filename is misleading: it contains O-B geometry. Preserve its original hash and use the explicitly named O-S file for the next design. Overall CAD bounds include the cable and jaw hardware.

See the [jaw geometry audit](geometry-audit.md) for the measured 11.4 mm imported opening, nominal-range rebase and sampled interference checks. The imported USD assets contain visual geometry only.

The local perception viewer had reached its configured 12-hour duration and exited cleanly. It was restarted through `scripts/start_demo.sh`; `scripts/check_demo.py` then passed with a recent matched image. Open [the local demo](http://127.0.0.1:8766/demo/) and [the existing mounted-tool video](http://127.0.0.1:8766/hardware/custom-gripper.html#mounted-motion). The local demo still shows the earlier stationary Pi tool; it is not a live view of this new candidate.

## First finger assembly

The [compact finger study](../../experiments/031-compact-fingers.md) adds bolt-on fingers and nominal mounting hardware. The assembled mechanism passes 23 sampled jaw openings; four dimensional cable coupons touch both pads without penetration. [Watch the Isaac review](../../docs/hardware/custom-gripper.html#compact-fingers). Motion is prescribed and the coupon stays fixed. Material selection, stiffness, pin retention, nozzle/adapter design and workcell clearance remain open.

## Mounted candidate

The [stationary mounting study](../../experiments/032-compact-mounted-layout.md) places the compact tool on the FR3 in a separate candidate scene. The old tool is inactive there. Bracket CAD fit and parked environment bounds pass their limited checks; fastening, stiffness, nozzle layout and full approach clearance remain open. No compact-tool physics or insertion is demonstrated.
