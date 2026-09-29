# Simulation to learned assembly — working plan

Updated 29 September 2026. This is the maintained execution plan; experiment records remain the evidence.

## Where we are

The local bench (038) and the separate FR3-mounted presented-cable lift (040) now pass their nominal positive, empty and feedback-dropout checks. The mounted positive run reaches 10.032 mm and holds contact; the empty case commands no lift; feedback loss latches a stop with 1.65 micrometres of measured additional tool travel over the observation window. These are three trials on one frozen configuration, not a reliability estimate or desk pickup. Cameras inspect motion but do not control it. Next is held-end camera inspection and a broader mechanics/stop test matrix.

## Immediate work

| Work | Status | Exit evidence |
|---|---|---|
| Archive rendered and native recorded-motion runs | Bench and mounted archives built; positive replays rendered in Isaac | Portable USD timeline, source trace comparisons, video, hashes and launch instructions |
| Sensor-only episode interface | Implemented; five boundary/timing/channel tests pass | Strict actor field allowlist, timestamps, finite values, explicit missing cameras; offline labels separate |
| Mounted grasp and 10 mm lift | Nominal three-case commissioning passed | Broaden placement, curvature, timestep and sensor-failure tests; complete collision/clearance review |
| Held-end inspection | Interface and saved-image failure review ready; fresh Isaac capture blocked by current Docker permissions | Independent image evidence of leading edge and slip; uncertainty triggers abstention |
| Sensor-guided free-space alignment | First learned motion target | Held-out metric position/angle errors against offline reference, bounded corrections |
| Contact insertion | Gated | Stable contact mechanics, collision/buckling/slip tests, stop/retract and seating evidence |

Every milestone update should change this table, link fresh evidence, and record failures. A new replay or video alone does not close a control gate.

## Seven skills and learning progression

1. **See the entrance.** Label entrance rims, cable leading edge and latch separately. Retain raw RGB, calibrated transforms and classical-CV outputs. Compare DINO features and segmentation against these measurements. Test glare, board rotation, partial occlusion and stale frames.
2. **Pick up.** Compare direct grasp and passive presentation. Learn only insulated-region grasps. Score lift retention, slip, tip exposure and modelled strain; physical damage limits remain unvalidated.
3. **Inspect and orient.** Move to a repeatable camera view. Estimate edge pose and deformation, ask for another view or regrasp when ambiguous.
4. **Align without contact.** First imitation-learning target: already-held cable, small relative tool corrections. Frozen perception, bounded action adapter and an independent geometric evaluator.
5. **Insert.** Start with bounded feedback control; collect successes and failures, then imitation and short-horizon RL. Observations include visual and contact history. Explicit actions include pause and retract.
6. **Verify and latch.** Separate policies and criteria. Command completion is not seating. Visual/contact evidence first; simulated continuity must not be presented as a physical electrical test.
7. **Recover.** Inject missed grasps, slips, shifted boards, missing/stale sensors and blocked entry. Measure detection, recovery and safe termination separately.

## Data boundary and timing

Actor, critic, reward and runtime stage transitions use deployable measurements only. Exact cable poses, mesh vertices, contact counterpart identities and socket ground truth live in a separate offline namespace for labels and scoring. A recorded-state playback may read these poses because it is an inspection artifact, never a controller.

Each episode needs run/config/source identity, schema version, clock domain, acquisition timestamps, sequence numbers, calibration IDs, measured robot/tool state, sensor validity and issued commands. Preserve raw images beside CV derivatives. Record action issue/application times separately when available. Never infer synchronized camera training frames from a presentation montage.

The initial bench export deliberately has no camera observations and no action-application timestamp. It is suitable for controller debugging, not vision-policy training. Ideal simulated force magnitudes must not be renamed measured six-axis wrench or tactile shear.

## Training and parallel execution

- Start with behavior cloning for bounded alignment; compare against the existing deterministic controller.
- Keep perception training, policy training and frozen-policy evaluation as separate jobs with immutable dataset/checkpoint IDs.
- Profile detailed cable environments before selecting worker count. Budget GPU memory for physics, image rendering and training; avoid resource contention with the live demonstration.
- Add RL only after repeatable contact and termination tests. Initially keep the critic observation-based too. Derive reward from observable progress/contact/seating evidence; use simulator geometry only to audit whether these proxies are misleading.
- Split complete scenes and episodes before augmentation. Reserve seeds, geometry variants and parameter combinations; adjacent frames must never cross train/test boundaries.
- Randomize within documented ranges: material stiffness/damping, friction, grasp offset, board pose, illumination, exposure, sensor delay/noise and frame loss. Distinguish measured ranges from exploratory assumptions.
- Evaluate success, collision, buckle/slip, false seating, abstention, recovery and duration with denominators and uncertainty. A higher mean reward is insufficient.

## Reproducibility library

Every accepted run should retain video, native timeline replay, raw sensor/action logs, offline labels, asset/source hashes, environment identity, command and outcome. Retain failed runs. Replay is sampled state playback; rerun means executing physics again. Do not promise bitwise equality across GPU/driver versions.

Native FR3 replay export is now implemented, using recorded robot encoders and offline tool/cable poses; trial 005 reopens with numerical checks passing. Initial and last scenes are captured on in-trial failure. Remaining capture gaps: action application timestamps, calibrated raw camera streams and fresh-build/positive rerun checks. Frozen physics bundles are implemented; an isolated rerun of failed trial 005 exactly matched recorded commands, angles and loads on the installed runtime. Historical bending traces contain tip poses only, so full-body motion cannot be reconstructed honestly from those traces.

Latest evidence: [039 — mounted lift commissioning and replay library](../../experiments/039-mounted-flexible-lift-and-library.md). Bench replay export and Isaac start/end rendering passed; 172 CPU tests pass. This earlier failed milestone is superseded by the nominal mounted-lift result in 040.

## Active follow-up: thin pads and mounted replay

[040 methods](../../experiments/040-thin-pad-collision-and-mounted-replay.md): body-pose audit rules out a gross initial tool-pose error. Lowering the convex cooking thickness floor did not change the failure. Analytic pad boxes derived from CAD remove premature open-jaw contact; run 006 held contact but timed out at 9.58 mm. Disabling sleeping in 007 did not change the plateau. Run 008 adds bounded encoder-error integration while retaining vendor joint friction and passes the lift/hold check. Mounted replay and sensor/action export are implemented. The failed-run bundle passed an isolated rerun check. The positive bundle is packaged; its fresh physics repeat remains pending.

Frozen commissioning suite: `fr3-flex-008`, `fr3-flex-empty-002`, `fr3-flex-dropout-002`; [machine-readable checks](../../docs/library/mounted-suite.json). The next deliverable is a calibrated held-end RGB stream and leading-edge/visibility measurements, with uncertainty causing abstention. Preserve the pad/encoder guard and keep Pi insertion disabled.

## Inspection preparation (041)

[041 record](../../experiments/041-held-end-inspection-preparation.md): RGB-only inspection contracts and a fixed-camera capture recipe are implemented. Saved-image colour-region review correctly abstains but fails to uniquely localize the terminal because adapter/tool surfaces share its colour. This does not close the leading-edge or slip gate. Five new tests pass; the full run has 177 passes and one existing server test blocked by sandbox socket restrictions. No new Isaac render or GPU inference ran. Next: execute the prepared raw-camera capture, review optics/clearance and label visible leading edges before fitting/evaluating a learned estimate.

## Camera capture completed (042)

Docker access was restored and the prepared camera rendered seven samples of the recorded mounted lift. [Review](../../docs/held-end-review/capture-001/index.html): raw RGB, colour masks, video, synthetic calibration, manifest and physics-disabled camera replay are archived. All seven frames were inspected; focus varies through the lift and the tool reference is cropped. The terminal reaches the top image boundary near the final pose. No leading-edge, slip or alignment gate has passed. All 178 CPU tests pass. Next: set a repeatable inspection pose, qualify framing/focus in simulation, and independently label visible leading edges before training. This capture does not rerun physics or measure ROS transport latency.
