# Simulation to learned assembly — working plan

Updated 29 September 2026. This is the maintained execution plan; experiment records remain the evidence.

## Where we are

The local presented-cable pinch/lift bench passed its positive, empty and feedback-dropout cases (038). It uses ideal pad signals and measured tool displacement. The mounted FR3 trial is still under commissioning: run 001 lost contact; run 002 timed out without the required bilateral load; its contact audit exposed solid screw-clearance envelopes interfering with the jaws. Run 003 excludes these nonphysical envelopes, established contact, began lifting, then lost contact at about 0.14 mm. Neither is a successful robot pickup or insertion. Cameras currently inspect these motion trials; they do not control them.

## Immediate work

| Work | Status | Exit evidence |
|---|---|---|
| Archive rendered and native recorded-motion runs | Bench archives built; start/end rendered in Isaac | Portable USD timeline, source trace comparisons, video, hashes and launch instructions |
| Sensor-only episode interface | Implemented; four boundary/timing tests pass | Strict actor field allowlist, timestamps, finite values, explicit missing cameras; offline labels separate |
| Mounted grasp and 10 mm lift | Commissioning | Positive retention, empty rejection, feedback-loss stop; full tool/robot collision audit |
| Held-end inspection | Next | Independent image evidence of leading edge and slip; uncertainty triggers abstention |
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

Current capture gaps to close: native FR3 replay export, complete initial-stage capture even on failure, action application timestamps, calibrated raw camera streams and portable physics rerun bundles. Historical bending traces contain tip poses only, so full-body motion cannot be reconstructed honestly from those traces.

Latest evidence: [039 — mounted lift commissioning and replay library](../../experiments/039-mounted-flexible-lift-and-library.md). Bench replay export and Isaac start/end rendering passed; 172 CPU tests pass. Next motion work is diagnosing grip loss under the mounted load, without reducing acceptance thresholds to obtain a pass.
