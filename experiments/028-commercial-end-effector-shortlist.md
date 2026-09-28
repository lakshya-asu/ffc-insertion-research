# Commercial vacuum and pinch hardware shortlist

Research date: 2026-09-28. This is a component selection proposal, not installed hardware or a validated cable gripper. Prices, delivery dates and exact variants need supplier confirmation.

## Proposed baseline

Evaluate a DH-Robotics PGE-5-26 electric parallel gripper with custom narrow compliant fingertips, plus a small vacuum pickup mounted alongside it. Vacuum lifts the insulated cable surface from the desk; the fingers then pinch a supported region behind the leading edge. The insertion end must remain exposed and visible. A fixture-assisted presentation/regrasp remains the comparison baseline.

This combines commercial actuators and vacuum components with custom tooling. A successful same-tool vacuum-to-pinch handoff is not established: cup offset, lower-finger clearance, cable curvature and camera occlusion must be checked in CAD and simulation before accepting the arrangement.

## Candidates and manufacturer evidence

| Component | Published specifications | Intended role / qualification gap |
|---|---|---|
| DH-Robotics PGE-5-26 | 0.8–5 N **per jaw**, 26 mm stroke, 0.4 kg, 95 × 55 × 26 mm, 24 V, standard Modbus RTU/RS485 and digital I/O | Low-force pinch candidate. Actual force at our custom fingers, closure overshoot, cable damage and slip remain unmeasured. Published force adjustment is not proof of direct tactile sensing. |
| Schmalz SUF 3 SI-55 M3-AG, 10.01.01.14292 | Nominal 3 mm cup; maximum diameter 3.7 mm; height 5.5 mm; M3 connection; theoretical suction force 0.3 N at −600 mbar on a flat, dry surface, without safety factors | Concrete small-cup geometry for a simulation candidate. Silicone variant is not automatically ESD-qualified. Seal leakage and peel retention on curved ribbon remain unknown. |
| SMC ZP oval family | Includes 2 × 4 mm oval pad | Alternative small elongated contact footprint. Exact material, adapter and part number remain to be selected. |
| SCHUNK EGK 25 | Published minimum gripping force 20 N | Lower priority for this delicate cable task; we have no evidence that this minimum is appropriate for our pad area and cable laminate. |

Sources:

- [DH official PGE page, with CAD downloads](https://en.dh-robotics.com/product/pge)
- [DH official brochure with PGE-5-26 dimensions and interfaces](https://en.dh-robotics.com/wp-content/uploads/2024/07/DH_Electric-Gripper-Brochure.pdf)
- [Schmalz exact SUF 3 component](https://www.schmalz.com/de-at/produkte/vakuumtechnik-fuer-die-automation-301607/vakuum-komponenten-301608/vakuum-sauggreifer-301609/flachsauggreifer-rund-301610/flachsauggreifer-suf-301611/10.01.01.14292)
- [Schmalz electronics suction-cup family and ESD material options](https://www.schmalz.com/en-de/products/vacuum-technology-for-automation-301607/vacuum-components-301608/vacuum-suction-cups-301609/suction-cups-for-the-electronics-industry-304938/flat-suction-cups-suf-305079)
- [SMC oval pad dimensions](https://www.smcworld.com/products/pickup/en-sg/vacuum_device/pad_style/oval_pad.html)
- [SCHUNK EGK 25 manufacturer listing](https://schunk.com/us/en/gripping-systems/parallel-gripper/egk/egk-25-ec-m-b/p/000000000001491756)

## Mechanical arrangement and sensing

FR3 wrist → verified adapter → electric gripper → replaceable narrow fingers. An offset vacuum pickup needs a validated handoff geometry; retractability may help but adds another actuator, mass and collision envelope. Do not assume FR3 plug-and-play compatibility. Verify flange drawings, fasteners, payload/inertia, electrical supply, strain relief and full wrist swept clearance.

Vacuum needs a pump or ejector, valve, suitable release/vent path, tubing and pressure measurement near the pickup. Vacuum pressure indicates seal conditions, not cable presence by itself: suction against the desk can also produce a seal. Require visual lift confirmation and retained-end inspection. Avoid strong release air jets until their effect on the loose cable is evaluated.

Pinch needs measured jaw position, device fault/status and preferably direct fingertip normal-force sensing. Treat any drive-current-derived force estimate as an estimate. Select a miniature force sensor after the fingertip envelope is established; a large tactile pad can hide the entrance or prevent close support behind the cable tip.

## ROS 2 contract to implement

Use an actuator adapter for the selected Modbus variant, with bounded commands for jaw width, closing speed and force setting. Publish timestamped measured jaw position, device state and errors. Publish direct force measurements separately from force commands/estimates. Vacuum control exposes enable/vent plus measured pressure and acquisition time. Abort, timeout and stale-sensor handling must have explicit outcomes; a command acknowledgment is not grasp success.

## Isaac qualification sequence

1. Import manufacturer CAD; distinguish collision geometry from visual meshes. Verify stroke, jaw direction, mass/inertia assumptions, mounting and visibility before dynamic trials.
2. Implement bounded jaw drive and compliant frictional pads. Test empty closure, thin-cable contact, overshoot, slip and release. Never represent a pinch as an unconditional fixed attachment.
3. Implement a declared reduced-order suction model with finite pressure response, leakage, contact/seal eligibility, shear/peel limits and release delay. These parameters are uncertain until bench measurement; CAD alone cannot provide them.
4. Compare direct pinch, vacuum-assisted pinch and fixture-assisted presentation using identical cable placements and perturbations. Score successful lift, retained orientation, exposed tip length, crease/strain proxies, visibility and failed-grasp detection separately.
5. Only advance to insertion after held-end stability and camera visibility pass. Demonstrate loss of seal, slip and missed pinch as well as success.

No new gripper has been installed in the current Isaac scene by this research note. The preceding E027 motion trials remain unloaded robot tests.
