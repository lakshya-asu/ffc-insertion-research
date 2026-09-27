# Contact stability and transport after handoff

E030 transfer-clearance passed a stationary suction-to-pinch handoff, but E030 direct-clearance failed shortly after the subsequent raise began. Its retained cable tip moved laterally during the vent hold, and the jaws eventually contacted each other (47.77 N impulse-derived peak). The tool self-contact gate stopped the run. A stationary retention result is therefore insufficient to qualify transport.

The revised guide-rod clearance remains necessary and is not reverted. Two further hypotheses are isolated:

1. **Compliant facing:** retain friction coefficients 0.5 static / 0.4 dynamic and the existing finite actuator limits; add an effective force-based jaw-facing contact spring of 10,000 N/m with 1 N·s/m damping. These are assumed contact-law values, not identified silicone properties. The contact mesh remains a rigid envelope and does not resolve pad strain. Test hold, then acquisition/venting/raise from the full direct run's pickup pose.
2. **Joint deactivation:** retain rigid jaw contact and disable the two vacuum joints on venting, instead of leaving zero-force free joints in the solver graph. This comparison now includes the corrected guide clearance and preload control that were missing in earlier deactivation trials.

The compliant-facing pure hold (`e033-compliant-pinch`) passed for 2 s. Its maximum recorded cable/tool contact-pair proxy was 0.325 N; minimum recorded separation was −16.4 µm. The earlier rigid pure hold (`e026-pure-pinch02`, 1 s) passed with a 0.438 N peak. Different run lengths mean this is a screening comparison, not a statistical stability claim. Both recordings and source hashes were verified. Vacuum handoff and transport require their separate trial results.

PhysX documents a force-based compliant contact spring and damper, with stiffness in N/m and damping in N·s/m. This is the implementation used here: [PhysxMaterialAPI schema](https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/latest/physxschema/class_physx_schema_physx_material_a_p_i.html). The material interaction with rigid contact is documented in [PxCombineMode](https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/_api_build/structPxCombineMode.html). A compliance model must be calibrated before predicting real grip forces or cable damage.

The retrospective tool-frame audit rejects both original rigid-contact handoffs as stable grasps: E030 transfer had 11.13 mm maximum free-tip drift, and E030 direct had 19.25 mm. Tool rotation was reconstructed from the official kinematics and recorded joint positions; the reconstructed patch location agreed with the recorded patch within 0.4 µm. This audit uses 10 Hz samples and does not bound intersample motion. New run-time phase gates require free-tip drift below 1 mm before connector contact. The earlier height-only PASS is retained as historical data and visibly qualified in the video index.


## Surface anchors and finite contact area

Neither the compliant-facing transfer nor the disabled-vacuum-joint transfer stabilized the grip; both stopped at empty-jaw contact after venting. A further model inspection found that suction body anchors were authored at the tool patch transformed into cable coordinates, including the initial air gap. That could put the anchor 0.2 mm outside the actual cable face and make the spring oppose upward motion into the shoe during pinch. Anchors now project onto the actual cable surface. A regression test constructs a rotated tool and verifies that the surface anchor is at the 0.15 mm cable half-thickness while the initial 0.2 mm capture gap remains as physical joint separation.

A separate contact-area hypothesis adds a 1.5 mm minimum effective torsional patch radius, bounded by a 2.5 mm radius inside each 5 × 8 mm cable-segment face. Linear friction and actuator force limits are unchanged. The preflight rejects patch radii outside this geometric bound. This represents finite-area rotational friction, not a welded grasp. Its pressure distribution and torque capacity remain uncalibrated. [PhysX documents the contact-radius parameters and their zero-radius behavior](https://nvidia-omniverse.github.io/PhysX/physx/5.5.0/_api_build/classPxShape.html).

E035 compares the corrected surface anchor without added torsional friction. E036 adds the finite patch. E037 full direct and fixture runs use the corrected surface anchor, finite patch, full-tip seating test and pre-insertion grip-drift gate. Their source snapshots are frozen and their actual results determine acceptance.


E035 surface-anchor transfer failed the new stability gate at 8.58 mm endpoint drift. E036 with finite contact area also failed (9.40 mm endpoint drift). Thus neither surface projection nor torsional patch friction is sufficient to stabilize this solver configuration; the surface-anchor geometry correction is still retained. E037 full runs were deliberately stopped after this matched benchmark failure.

The next comparison keeps physical parameters and uses TGS with zero velocity iterations and external forces applied each internal iteration. It is tested against both the static nonlinear reference and transfer, rather than judged solely by task success. NVIDIA documents poor D6-drive behavior with TGS velocity iterations and recommends few or zero velocity iterations for relevant force-sensing configurations: [known limitations](https://nvidia-omniverse.github.io/PhysX/ovphysx/0.6.3/guides/limitations.html), [simulation control](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/simulation_control/simulation_control.html).


## Solver screening result

PGS with 255 position iterations, four velocity iterations and 8 kHz physics produced 46.0819 mm full-span static sag against the independent nonlinear reference of 46.1317 mm (ratio 0.998921; −0.108%). The settled link-center shape overlays the independent reference, and the declared motion/shape gate passes. Media decoding and clock/source audits passed. This uses the same nominal physical cable parameters and benchmark-only body drag. The corresponding TGS profile with zero velocity iterations and per-iteration external forces gives 51.1462 mm (ratio 1.10870). These are comparisons of numerical profiles, not changes to material stiffness.

Because the first PGS record became motionless, an awake repeat disables sleeping and stabilization. PGS transfer and complete direct/fixture runs are also underway. Static agreement does not by itself qualify contact dynamics or handling.


## Selected PGS transfer result

E039 PGS transfer passed acquisition, lift, deployment, pinch, vacuum release and a further raise. The independent 10 Hz tool-frame audit found a maximum 1.065 µm tip drift (0.095 µm during venting; 1.065 µm during the subsequent raise). The all-step maximum cable/tool contact-pair estimate was 0.2521 N, with no unintended arm/tool or tool/tool contacts. Its 14.73 s simulation produced 442 decoded frames and eight verified stage clips. This qualifies this deterministic handoff-and-raise experiment, not a hardware reliability rate.

The selected default is PGS, 255 position iterations, four velocity iterations and 8 kHz physics, with the original finite-force rigid jaw contact. Added compliant facing and torsional patch friction are not enabled in the selected profile. The corrected surface suction anchors and physical guide clearance remain enabled. E043 repeats the awake static test against a declared ±1% numerical reference tolerance and passes (46.0810 mm sag, −0.110%). It has no cable/environment or cable/self contact pairs.

E044 repeats all seven tool-exercise stages under the selected profile. Both measured actuator strokes are approximately 18 mm; 300 video frames and seven clips were decoded and verified.


## Full direct sequence

E040 direct PGS passes all 16 phases from an initially ungrasped cable on the desk to the open connector. The final independent audit requires every tip corner beyond 3.7 mm and inside the slot; the recorded tip is at 3.80105 mm depth, −0.111 µm lateral error and +1.323 µm height error. Maximum sampled tool-frame tip drift through venting/raise/transport is 1.211 µm. No fixture contacts occur, so this run does not rely on the passive support. All-step cable/tool contact-pair peak is 0.2518 N, with no unintended tool or arm/tool contact. Cable/connector contact force is zero: the assumed clearance slot and privileged alignment do not represent terminal-spring insertion resistance.

The trial takes 41.76 simulated seconds, with 334,080 requested and observed physics callbacks. All 1,253 frames, 16 stage clips and 20 frozen source hashes verified. This is a single deterministic geometric baseline, not an empirical success rate, electrical connection or real-robot demonstration.

The final independent grip audit additionally covers connector alignment and slot approach, extending the maximum sampled pre-insertion drift to 1.315 µm. The task remains qualified; this tighter audit changes no simulated motion.


## Full fixture sequence

E040 fixture PGS also passes the independent geometric task audit. All 23 stages complete, including tail-first shelf placement, suction release, independent pinch regrasp and transport. The contact record contains 26 cable/shelf pairs (largest per-pair impulse-derived force 0.00706 N), confirming physical support rather than a hidden attachment. Final full-tip depth is 3.80099 mm, lateral error −0.231 µm and height error +1.263 µm. Maximum sampled pre-insertion tool-frame tip drift is 1.929 µm. Cable/tool contact-pair peak is 0.2347 N; cable/connector contact remains zero in the clearance-slot model. No unintended arm/tool or tool/tool contact exceeds the declared gate.

The 66.789 s simulation records 534,312 matching physics callbacks, 2,004 decoded video frames and 23 verified clips. Direct takes 41.760 simulated seconds and 16 stages under the chosen controllers. Use direct as the nominal initial baseline, while retaining fixture handling for future matched variation tests. These two single trials establish neither a reliability ranking nor hardware cycle time. `scripts/compare_strategies.py` produces the measured phase-duration plot and comparison JSON.
