# 037 — Fitting an assumed cable world in Isaac

The reference world is the declared elastic model in `config/cables/world-model-v1.json`. It is sufficient to develop a simulation proof of concept without waiting for real material measurements. This experiment fits numerical behavior to that world and evaluates static bending; it does not identify the purchased cable's mechanics.

## The comparison

The frozen target archive has 30 fitting cases and 30 reserved validation cases. Fitting spans are 24 and 32 mm; reserved spans are 28 and 40 mm. Each uses five distributed loads and three stiffness assumptions. Width, thickness and density come from the source-aware profile. The first whole 2 mm segment is fixed, so the independent nonlinear hinge-chain reference uses the same boundary condition as Isaac.

The cable advances under native Isaac/PhysX dynamics. Object poses are measured only for offline evaluation. No controller commands cable transforms or follows reference positions. Red curves are non-colliding visual guides, shifted sideways beside the ribbon so the reference centerline remains visible; their X/Z geometry is unchanged. The robot, gripper and insertion controller are absent from this isolated benchmark.

The declared case criterion is at most 5% error in final tip sag, plus final 50 ms motion below 1% of reference sag. A separate saved-stage audit compares every segment endpoint with the reference centerline. The shape audit is additional evidence, not a substitute for the declared tip and settling checks.

## What changed

Increasing spring gains alone did little to the original error. The next representation uses a fixed-root articulation with three angular degrees of freedom at each internal joint. A planar hinge-only diagnostic gave essentially the same out-of-plane result, so locking the other angular degrees of freedom did not solve the discrepancy.

Reducing the joint viscosity multiplier from 1 to 0.001 improved the 32 mm nominal case from 1.924 mm sag to 0.413443 mm, against a 0.415083 mm reference. The subsequent fitting batches use multiplier 0.00001. Artificial 20/s body drag assists settling. These terms are numerical settings for this static approximation, not identified material damping; dynamic response remains to be modeled and evaluated.

| Local run | Change | Final sag | Outcome |
|---|---|---|---|
| gain-fit-001 | Maximal coordinates, gain ×16 | 4.505661 mm | Gain alone did not repair the fit |
| gain-fit-002 | Articulation, gain ×1 | 1.924169 mm | Reference disagreement |
| gain-fit-003 | Articulation, gain ×8 | 1.731968 mm | Reference disagreement |
| gain-fit-004 | Planar hinge articulation | 1.924155 mm | Reference disagreement |
| gain-fit-005 | Force-update diagnostic | No result | API instrumentation error; retained |
| gain-fit-006 | TGS, zero velocity iterations, external forces each iteration | 1.894281 mm | Reference disagreement |
| gain-fit-007 | PGS articulation, viscosity ×0.001 | 0.413443 mm | Single nominal case passed |

The force-update and drive investigations followed NVIDIA's [documented physics limitations](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.0/dev_guide/guides/current_limitations.html). Those documented limitations inform hypotheses; the actual comparisons determine whether a setting helps this model.

## Broad fitting and retained failures

`bending-fit-001` completed its physics and recording, then failed to save its report. The serialization path used a NumPy boolean. The corrected runner converts acceptance results to native booleans, saves traces and the final stage before serializing the report, and writes a failure traceback before closing Isaac if an exception occurs. The original recording is retained and is not used as numerical fit evidence.

`bending-fit-002` evaluated gains 0.98, 1 and 1.02 at 0.125 ms. Each candidate passed 26 of 30 fitting cases. The failures were the stiffest material at the two smallest loads, at both spans. Selecting by relative RMSE favored gain 1.02, but its worst relative error was still 41.58%. That unsuccessful decision is preserved in `outputs/bending-selection-001/selection.json`; it is not a passing release.

For the difficult 32 mm, stiffness ×4, load ×0.25 case, `gain-fit-008` disabled articulation sleep and stabilization. `gain-fit-009` also expressed masses, inertias, spring/damping coefficients and force limits in consistent gram-based units. Both measured 0.037077 mm against a 0.025946 mm reference. Neither change resolved this outlier.

`gain-fit-010` reduced the timestep from 0.125 to 0.03125 ms at 255 position iterations. Sag became 0.025781 mm, a 0.637% error, and the case passed. A faster PGS diagnostic with 32 iterations (`gain-fit-011`) missed the criterion at 6.943% error. TGS diagnostics using zero velocity iterations and external-force updates also missed it: 7.357% at 32 position iterations (`gain-fit-012`) and 105.207% at 128 (`gain-fit-013`). No faster setting from these diagnostics is accepted as equivalent.

The refined profile therefore keeps PGS, 255 position iterations, 8 velocity iterations, a 0.03125 ms timestep, gain 1, viscosity multiplier 0.00001 and artificial body drag 20/s. Articulation sleep and stabilization are disabled. The material reference is unchanged.

The complete refined fitting run is `bending-fit-003`. Before reserved results were available, the numerical profile was frozen in `outputs/bending-selection-002/selection.json`, based on fitting-only diagnostic evidence. `bending-validation-001` then ran concurrently with the remaining fitting computation. Validation was not used to choose or alter settings. This is a pre-registered profile, rather than a claim that the broad fitting evaluation had already finished at registration.

## Small-angle measurement correction

The initial refined reports (`bending-fit-003` and `bending-validation-001`) each recorded 29/30 passes. Inspecting the remaining failures exposed an endpoint-measurement defect: the float32 quaternion scalar could round to 1 while its vector part still encoded a small rotation. Converting that quaternion through `Gf.Rotation` lost part or all of the angle. Using the saved USD transform matrix reproduced the problem, so the first matrix-based shape audits were not independent protection against it.

The corrected calculation normalizes the quaternion in float64 and rotates the endpoint offset using vector products. A regression test retains the exact near-identity case, compares with SciPy, and checks general orientations and invalid inputs. Rescoring the original saved USD poses gives worst full-centerline errors of 0.315% (fit) and 0.273% (reserved spans), normalized by each reference tip sag. This is an audit of the final shape, not recovered evidence about the settling window: the original dense traces did not contain raw orientations.

Complete repeats `bending-fit-004` and `bending-validation-002` therefore use the same pre-registered physics settings and corrected measurements. They archive every sampled position and quaternion, including every physics step during the last 50 ms. `scripts/audit_bending_measurements.py` independently recomputes sag and settling using SciPy and refuses missing samples or disagreement with the report. The original reports and failed trials remain available. These are repeated reserved cases, not a newly unseen evaluation set; no physics parameters were selected from the rescored validation results.

## Completed corrected runs

Both repeats completed with exactly 32,000 physics callbacks and 1.0000000475 simulated seconds. All 60 cases passed the unchanged criteria: at most 5% final tip-sag error and at most 1% of reference sag in final-50-ms movement. The independent SciPy rescore agrees with the runtime measurements and checks all 1,601 settling samples in each case.

| Split | Passed | Largest tip-sag error | Largest absolute tip error | Largest full-centerline error / reference tip sag |
|---|---:|---:|---:|---:|
| Fit, `bending-fit-004` | 30/30 | 0.2944% | 1.8594 µm | 0.3149% |
| Reserved, `bending-validation-002` | 30/30 | 0.2244% | 4.2985 µm | 0.2732% |

Maximum errors in different columns may occur in different cases. These results qualify the declared static short-span response family; they do not establish full-cable/contact mechanics or physical accuracy. The two Isaac application runs took roughly 635 and 792 wall-clock seconds while overlapping on this workstation, including startup and rendering. That is not a training-throughput benchmark, but it makes solver cost an explicit concern for the later dataset pipeline.

The website publishes the movies, all 30 reserved close-ups, reference curves, original reports, corrected reports, raw pose archives, independent audits and a downloadable final Isaac stage. A desktop/mobile browser check covers all 30 image selections and video playback. No policy-training dataset is released. The [contact qualification plan](../research/cables/contact-qualification.md) defines the next mechanics benches.

## Review and data boundaries

`scripts/isaac_fit_bending.py` records the fitting/validation split, numerical settings, target hash and source hashes before launching Isaac. Validation requires a saved selection and checks the selected gain and numerical settings before executing. `ffc.bending_fit.select_gain` refuses validation data and unequal candidate case sets. Tests cover those boundaries.

The refined batches count actual physics callbacks and their accumulated time. Each must produce exactly 32,000 callbacks and one simulated second. The reserved image pass also checks that rendering its settled-state close-ups adds no physics steps.

Live frames are human review images, published through `ffc.lab_feed`. They do not enter the ROS control pipeline. Saved validation close-ups show final simulated states, not real photographs. The presentation replay can be slowed for readability; its caption identifies the playback change.

Static short-span agreement is one component of the assumed world. The 200 mm profile, stiffeners, directional twist, release dynamics, desk/finger friction, vacuum and connector contact remain separate qualification tasks. No insertion-policy dataset is released by this experiment.
