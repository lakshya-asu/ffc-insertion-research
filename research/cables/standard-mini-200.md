# The cable we are building around

We are targeting the Raspberry Pi Camera Cable Standard–Mini, 200 mm, with the revision-two outline described in PCN 36. The procurement requirement is to match that product and outline. Pin count alone is not enough. All gripper candidates remain open.

The [machine-readable evidence profile](../../config/cables/rpi-camera-standard-mini-200-rev2.json) is the parameter source for the revised fixture experiment. Every value is marked as manufacturer-published or assumed. Each run archives the profile and its SHA-256 identity. Earlier runs retain their original geometry and assumptions.

## What the manufacturer tells us

| Property | Value | Evidence |
|---|---|---|
| Product length | 200 mm | [Product page](https://www.raspberrypi.com/products/camera-cable/) |
| Construction | Shielded camera FPC | Same product page; exact shielding stack is undisclosed |
| Mini interface | 22 contacts, 0.5 mm pitch, 11.5 mm width | [Hardware documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html) |
| Standard interface | 15 contacts, 1 mm pitch, 16 mm width | Same hardware documentation |
| Revision-two outline | Most of the cable narrowed to 11.5 mm; wider transition at the 15-pin end | [PCN 36](https://pip-assets.raspberrypi.com/categories/1267-pcn/documents/RP-009201-PC-1-15%20to%2022%20pin%20camera%20cables%20Rev%202.pdf) |

[PCN 48](https://pip-assets.raspberrypi.com/categories/1267-pcn/documents/RP-010539-PC-1-PCN%2048_%20Zero%20Camera%20Cable%20form%20factor%20and%20manufacturing%20change.pdf) covers the separate 38 mm and 150 mm Zero camera cables. Its manufacturer change does not identify the laminate of our 200 mm cable. Likewise, a Standard–Standard cable drawing is not a mating drawing for this product.

## What still needs an explicit assumption

The reviewed sources do not give the full laminate, copper trace dimensions, shield construction, adhesives, directional bending stiffness, damping, friction or damage limits. They also do not establish all terminal dimensions or the contact-face relationship. We can continue simulation with declared ranges; we cannot claim that these ranges reproduce the exact purchased cable.

The initial equivalent model uses a 0.14 mm body, 0.30 mm reinforced ends, 3 GPa equivalent modulus and 1,800 kg/m³ equivalent density. These are sensitivity-study inputs. The profile includes exploratory bounds, which are **not manufacturing tolerances or probability distributions**. Its 4 mm exposed contact length, 6 mm stiffener length and width-transition location are also assumptions.

The fixture runner now models both terminal regions and the changing width. Its gold contact strips are visual geometry, not an electrical simulation. The selected contact-face convention needs verification. The model is a segmented equivalent ribbon, not a resolved laminate FEA model.

## How we will establish useful physics

First, check the solver against an independent calculation with the same boundary conditions. For a homogeneous strip, out-of-plane rigidity is `EI = E w t³ / 12`; the equivalent hinge stiffness is `EI / segment_length`, converted to the angular units expected by USD. This is a numerical verification problem: changing the material modulus to hide a solver error would invalidate it.

The isolated benchmark fixes the first full segment of a 32 mm strip and compares the remaining chain against a nonlinear static energy minimization. It records a video and the final 50 ms of tip motion. The declared single-case limit is 5% relative error, with final motion below 1% of the predicted sag. Passing one case is insufficient: timestep and spatial refinement must also agree. Artificial drag used for settling is identified separately from material damping.

Next come bending in both directions, twist, residual curl, recovery after loading and contact with the desk and pads. A layered shell or beam model must represent the actual trace/shield layout when those data become available; a single isotropic modulus cannot establish all these responses. We need independent reference cases for each solver representation before transferring its parameters into task rollouts.

Contact tests then cover frictional slip, pad compression, suction seal acquisition/leakage, lift, release and transfer to the fixture. Connector tests cover edge collision, skew, buckle, blocked entry and retract. An undeformed cable or a cable attached by a hidden constraint does not count as successful pickup. Damage cannot be scored credibly until a damage model and its thresholds have evidence.

Physical samples remain useful for later identification, but are not a prerequisite for this simulation-first work. Our initial claim will be success across a documented assumed parameter envelope, with unresolved physical properties stated. Exact-product fidelity requires stronger evidence.

## Dataset and learning sequence

1. **Freeze a qualified simulation profile.** Record cable profile hash, code revision, solver settings, geometry, camera calibration, tool and sensor model versions. Mechanical verification and contact tests must support the particular skill being collected.
2. **Collect sensor-driven demonstrations and failures.** Record synchronized RGB, calibration, frame age, robot/tool measurements, available force/tactile/pressure signals, bounded commands, stage transitions and failure reasons. A proposed sensor without a functioning model supplies no pretend measurement.
3. **Keep two separate data products.** Policy observations contain deployable measurements only. Offline labels contain simulation geometry, segmentation and independent scoring. Simulator object poses must never become controller inputs, privileged critic inputs, online rewards or stage-transition shortcuts.
4. **Split before fitting anything.** Keep complete episodes and related scene families in one split. Hold out geometry/material settings, lighting, occlusion, initial curvature and tool/camera perturbations. Fit normalization on training data only. The immutable final test set is not a tuning set.
5. **Train perception, then action.** Extend the current OpenCV/DINOv3 work to entrance rims, cable leading edge and latch state, with uncertainty and explicit abstention. Then fit a small sensor-conditioned imitation baseline for one qualified skill. Compare it with the bounded sensor-feedback controller using identical held-out episodes.
6. **Add constrained residual RL only after a stable baseline.** Keep action bounds, freshness checks, stop/retract and force limits outside the learned policy. Rewards and termination use deployable observations. Evaluate failure detection, stop latency and recovery as well as success rate. Report trial counts and uncertainty.
7. **Compose the seven skills.** Pickup, held-end inspection and alignment each earn their own release. Insertion and latch manipulation follow only after their contact models pass. A segmentation dataset can exist earlier; it does not qualify an insertion-policy dataset.

This sequence is a plan, not a claim that the action-training pipeline already runs. Current mechanics failures keep that dataset release closed. The source profile and numerical experiments are implemented now; qualified contact rollouts and policy training follow their acceptance tests.

### Implemented release check

`scripts/review_skill_dataset.py` audits hash-pinned review evidence for bending convergence, contact, pickup and the sensor boundary. Insertion additionally requires insertion and recovery evidence. A missing report, changed bytes, wrong cable profile or failed acceptance review produces a nonzero exit code. This verifies provenance and review status; it cannot establish that a review's scientific claims are correct.

The checked-in `config/cables/pickup-release-review.json` intentionally contains no passing evidence. Running it exits with code 2 and lists the four missing gates. This new check is an entry point for the future skill pipeline; it is not connected to the existing static segmentation training scripts. It does not imply that captured episodes, dataset splits or training have been implemented.

```bash
.venv/bin/python scripts/review_skill_dataset.py \
  --manifest config/cables/pickup-release-review.json \
  --evidence-root outputs --output outputs/NEW-release-review.json
```
