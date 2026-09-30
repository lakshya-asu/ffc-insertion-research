# Seven skills, with evidence and remaining work

Updated 29 September 2026. The immediate objective is a complete **simulation proof of concept**. Real-camera and physical robot tests are deferred. No current result qualifies deployment on a real robot.

The system has useful components and several successful subsystem experiments. It does not yet perform the seven skills end to end with the new tool and Raspberry Pi connector.

## 1. See the entrance — synthetic baseline established

**Built and tested:** Pi 4 and Pi Zero 2 W scene variants; camera-cable geometry and labelled entrance features; macro-camera optics and mounting studies; separate upper/lower entrance rims, cable leading edge and slider labels; classical image preprocessing; a frozen DINOv3 feature model with a trained task head; held-out synthetic evaluation; native ROS 2 RGB and acquisition identity; stale-frame checks; matched RGB/prediction review on the live site. Image-space alignment measurements are available on a retrospective evaluation set.

**Still needed:** re-run visibility tests with the compact mounted candidate progressively approaching; fresh independent scenes covering glare, exposure, pose and partial occlusion; calibrate confidence/abstention; establish camera-to-robot and tool-to-camera transforms in the composed dynamic scene; test metric pose/depth estimates against offline references. A rim mask and a pixel centroid are not a verified insertion frame. The static renderer currently averages roughly 1.2 fps, so the 500 ms freshness contract correctly rejects stale intervals.

**Gate to advance:** entrance boundaries and the cable leading edge remain observable throughout the intended approach, or the system detects ambiguity and requests another view. Accuracy must be evaluated against task clearances, not only IoU.

## 2. Pick up without damage — tool mechanics under commissioning

**Built and tested:** the earlier concept uses authentic PQ12 actuator surfaces, custom printed clevises/carriage/shoe, metal frame and finger, and vacuum-cup envelopes. The CAD has 47 components. A bounded 19-pose solid check informed an above-plane stow position. The new Isaac integration adds a provisional flange adapter, three rigid bodies, two physical prismatic axes, explicit payload assumptions and a mounted empty-tool commissioning sequence. See the mounted-motion evidence for current results.

**Still needed:** complete the compliance and load-cell force path; choose guide/bearing fits and retention hardware; qualify the flange adapter and inertia; represent actual pad/cup compliance; simulate pressure, seal/leak and loss-of-vacuum observations; calibrate or bound cable bending/torsion and contact; represent actuator backlash, friction, electronics and duty cycle. Compare direct pickup with a passive presentation fixture. Test randomized placement, curvature, front/back facing, slip, exposed tip and damage proxies.

**Gate to advance:** a pickup is detected from deployable observations, remains stable during lifting and preserves an inspectable cable end. No simulator attachment flag may stand in for grasp verification.

The compact PGEA candidate now has a separate mounted geometry study (032) and a sensor-driven rigid-sample pinch bench (033). The latter tests bilateral load gating, a 10 mm lift, empty-grasp rejection and injected load-signal loss. It is not full cable pickup, camera-guided motion or a validated PGEA actuator model.

The offline footprint review (034) now excludes the exposed-contact strip on both faces. It rejects the near-tip stiffener placement for the current pads and identifies farther-back insulated-body placements for subsequent mechanics tests. This is flat geometry only, not a damage or buckling result. All commercial and custom tool candidates remain open in `design/compact-tool/options.md`.

Side-entry handoff study (035) adds fixture CAD, a rotated-pad check and conservative swept desk/fixture clearances. Four unanchored ribbon-settling runs expose large timestep sensitivity: nominal tip drop changes from 4.689 to 2.508 mm when the timestep halves. The new cable dynamics are therefore unqualified. Isolate and resolve numerical bending/contact error before treating a pickup run as credible material behavior.

Experiment [038](experiments/038-flexible-cable-motion.md) adds an exploratory full-profile flexible-cable contact pilot: a presented end rises 9.973 mm while the tool rises 9.992 mm, using local pad/encoder feedback. Empty and signal-dropout trials refuse or stop the lift. The tail remains supported; contact convergence, whole-tool clearance, camera verification, FR3 mounting and ROS integration remain open. This is partial step-2 evidence, not a completed pickup skill.

Experiment [040](experiments/040-thin-pad-collision-and-mounted-replay.md) adds the compact tool on the FR3 with a full presented cable. The nominal positive run reaches 10.032 mm and holds contact. Matched empty and feedback-loss trials pass their controller checks; the latter shows 1.65 micrometres of additional measured lift after detection. An offline material-point audit supports retention in this simulated run. This is not desk pickup, camera verification or material/damage qualification. [Videos, native replays and source bundles](https://lakshya-asu.github.io/ffc-insertion-research/library/#mounted).

## 3. Inspect and orient the held end — metric inspection pilot; alignment blocked

**Built:** inspection-view concept, cable-end labels, camera observation infrastructure and a mechanically deployable finger concept.

**Still needed:** dynamic held-cable scenes; inspection poses and their visibility/clearance; leading-edge and orientation estimation; uncertainty and visible-deformation measurements; slip detection from vision plus tactile/pressure observations; active second-view and regrasp policies. A stable, genuinely simulated grasp is a prerequisite.

**Gate to advance:** estimate a held-end frame with bounded uncertainty, confirm stable retention and refuse the approach if orientation is ambiguous.

Experiment [046](experiments/046-zero-cell-lift-and-stereo.md) combines a fresh FR3 lift with the Zero board/socket and native stereo RGB inspection. The presented cable rises 10.032 mm. Its fresh edge estimate is 95.3 µm from the offline reference, exceeding the assumed vertical clearance despite a small reprojection residual. No orientation correction or insertion is authorized. Earlier 22–28 µm development-frame results did not carry over. Clear-side entrance views are available; metric entrance pose and face verification remain open.

## 4. Align before contact — offline measurements and motion contracts exist

**Built and tested:** image-only edge fitting, explicit abstention on absent/hidden features, typed observation contracts, a non-executable motion-proposal interface, one-command-at-a-time and stale/cancel logic. Actual Isaac unloaded FR3 joint probes and cancellation have passed. Mounted-tool commissioning extends that foundation.

**Still needed:** identify the image-to-joint Jacobian from sensor observations; qualify a collision-free corridor for this larger tool, macro camera and PCB; verify tool and camera extrinsics; connect perception to short, bounded motion with a fresh post-motion observation; score physical alignment errors independently. The dry-run interface is still `executable: false`.

**Gate to advance:** closed-loop alignment reduces independently scored error to a tolerance derived from the selected cable and opening, while preserving clearance and observation freshness.

## 5. Make contact and insert — channel pilot tested; Pi skill not implemented

**Existing evidence:** historical generic assembly simulations exercised robot motion and contact, but used privileged state and different geometry. They do not validate the current Pi task or the new tool.

Experiment [043](experiments/043-insertion-contact-pilot.md) adds a prealigned, sensor-guarded side-entry channel bench. The sideways tool feeds the cable into the assumed channel, retracts at a blocked mouth and stops after feedback loss. Three sampled CAD/fixture bounds checks pass. Original inline tool arrangements fail that clearance review and remain archived. This is no FR3 approach, camera alignment, exact Pi contact model or seating verification.

**Still needed:** qualify the cable/connector contact model; implement deployable force/tactile/pressure channels; develop bounded contact transitions; test edge hits, skew, buckle, slip, blocked entry and false force signals; establish stop/retract behavior. Choose force thresholds from modeled material/contact studies with sensitivity ranges, then identify them physically later. Do not treat guessed values as damage limits.

**Gate to advance:** detect the contact state and complete insertion under disturbances, or stop and retract without increasing the failure. Independent offline geometry may score insertion; it must not guide the policy.

## 6. Verify seating and operate latch — separate skills still needed

**Built:** connector/slider geometry and preliminary static slider segmentation. This is not seating or latch-operation evidence.

**Still needed:** a dynamic latch with stops, friction and resistance assumptions; seating observations; a separate latch-manipulation action; explicit partially seated and mislatched negative cases. Electrical continuity/function is an additional future lab measurement. A synthetic continuity model must state its assumptions and must not become a truth shortcut for motion control.

**Gate to advance:** completion is supported by independent evidence of seating and latch state. Reaching a commanded position is never sufficient.

## 7. Recover and repeat — infrastructure fault checks, not task recovery

**Built and tested:** observation shape/timestamp/calibration/rig contracts; stale-frame and dry-run cancellation tests; measured unloaded joint cancellation. Mounted arm and tool cancellation have also passed separate simulations with recorded joint feedback.

**Still needed:** task-level missed-grasp, seal-loss, slip, shifted-board, tracking-loss and blocked-insertion cases; tested retry/regrasp/retract limits; safe terminal states; reproducible randomized trials with failures preserved and reported. Add model/policy versioning and held-out seeds to every task evaluation.

**Gate to advance:** detect failures, choose a bounded recovery or safe stop, and report per-stage and whole-task success with uncertainty and failure categories.

## What the next milestone should deliver

Experiment [036](experiments/036-cable-evidence-and-thin-bending.md) pins the exact cable product/revision
and adds a source-aware parameter profile. Three isolated thin-body bending checks fail their independent
reference comparison. Resolve that numerical error before collecting a qualified pickup dataset.
The [cable dossier](research/cables/standard-mini-200.md) records assumptions and the staged learning plan;
real material identification remains deferred and no exact-product physics claim is made.

Experiment [037](experiments/037-assumed-world-fit.md) extends those checks to 30 fitting and 30 reserved short-span bending cases. It preserves the numerical trials, fixes a small-angle measurement defect, and repeats the frozen physics profile with auditable raw poses. The corrected repeats pass 30/30 fitting and 30/30 reserved cases, independently rescored from raw poses. This addresses static out-of-plane bending only. The [contact qualification plan](research/cables/contact-qualification.md) specifies the full-profile, support, friction and flexible-grasp benches that follow.

The next complete skill milestone is sensor-verified flexible-cable pickup in simulation. Start from the contact-exclusion review (034), compare integrated suction-to-pinch motion with the passive presentation fixture, and qualify swept clearance and cable deformation. Then connect vacuum/pinch observations and dynamic compact-tool motion to the camera/ROS pipeline. Re-check held-end visibility and approach clearance before enabling the Pi approach. The earlier mounted empty-tool motion (030) and local pinch bench (033) remain separate evidence, not an integrated pickup result.

Training comes after those observation/action interfaces and resettable task mechanics are credible. Begin with scripted bounded controllers and demonstrations; compare imitation/vision-action learning and reinforcement learning using identical observations, actuator limits and independent task scoring. Simulator truth can provide offline labels and evaluation, not hidden task-state input to the deployed policy.

Immediate next milestone: integrate a calibrated held-end inspection view, estimate the leading edge and detect ambiguity/slip from images. Broaden the mounted mechanics and stop tests alongside it. The maintained [learning and simulation plan](research/learning/PLAN.md) sets the sequence: inspection → sensor-guided alignment → demonstrations → contact RL after its mechanics gates.

Inspection update [041](experiments/041-held-end-inspection-preparation.md): sensor-only appearance diagnostics and a raw camera capture recipe are ready. The colour baseline fails terminal localization on saved views and abstains. Docker access was restored in 042 and seven camera frames are now archived with video and native replay. Focus and framing still need adjustment; no leading-edge, slip or alignment claim has been added.

Insertion update (043): [videos, native replays, sensor traces and frozen rerun bundles](docs/library/insertion.html) are ready. Next is contact sensitivity and socket-specific geometry, while inspection/alignment gates remain open.

Socket geometry research 044: supplier reference drawings and mating dimensions are recorded in `research/connectors/pi-socket-contact.md`. Research 044 alone did not advance Step 5; its implementation follows in experiment 045 below.

Experiment 045 integrates a supplier-family side-entry reference into the local contact bench: separate guides, open slider and spring contacts. A paired 0.15 mm offset comparison entered with tapered guides and retracted with square entry. Full board/FR3 integration, seating and hardware-calibrated contact remain open. See `experiments/045-socket-contact-reference.md`.

Corrected 045 contact ramps reach the backstop, with up to 4 micrometres of penetration flagged by strict scoring. The blocked-mouth run retracts with the tip outside. Step 5 has local entry/retract evidence, but contact convergence and seating remain open.

Compact-tool design update: the [micro-flexure candidate](design/micro-flexure/README.md) provides a 30 × 28 × 24.7 mm suction/pinch head concept. This is geometry and analytical screening, with prescribed jaw playback. Steps 2–5 still require tool-specific mechanics, handoff, inspection and insertion qualification; no seven-step gate is closed by the new CAD.

Visual review update: the compact gripper now has a reproducible Isaac RTX path-traced presentation workflow with fine meshes and declared edge-finishing/material proposals. Static rendering does not advance any mechanics or control gate.
