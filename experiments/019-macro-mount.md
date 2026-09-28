# Camera mount and tool clearance

The simulation design now uses the existing Basler/Kowa pair at 45° elevation and zero yaw, 100 mm lens-front working distance. Selected optical profile: `config/macro-mounted-camera.json`. This supersedes the 25° mounting position for the full-tool scene; the earlier optical study remains preserved.

The front is approximately 70.7 mm above and 70.7 mm horizontally from its focus target. The camera is supported from the negative-Y side of the PCB, opposite the offset tool and wrist. A 30 mm square-section upright and cross-arm, diagonal brace, foot, tilt-saddle envelope and camera adapter envelope share a baseplate with the PCB supports. A black box reserves space for a USB service loop and strain relief; it is not the literal cable shape. Fasteners, slots and structural stiffness have not been modeled or qualified.

The hardware interface reference is Basler adapter 2200002593:
https://www.baslerweb.com/en/shop/tripod-mount-ace-2-and-racer-2-s/
https://assets-ctf.baslerweb.com/dg51pdwahxgw/4Lsm5nENLPMcbCYINfCfjf/348ee5ec08494900231490aa79ab34aa/DG00228502000_Tech_Spec_for_SAP_2200002593_EN.pdf

It has three camera attachment screws and a 1/4-20 mounting interface. Our 51×29×8 mm adapter bounding envelope is intentionally larger than the latest drawing's 44.1×29×6 mm plate. It is not a hole-pattern or fabrication drawing. No parts were purchased.

## Why the earlier layout changed

The small finger proxies did not represent the existing complete offset tool. The present check extracts all 11 tool cube/cylinder bounds, retaining the housing, offset neck, stem, upper support, rail, vacuum patches, carriage, lower jaw and guide structure. Both directions of the nominal 18 mm deployment and 20 mm clamp travel are conservatively added to the moving subassemblies. These allowances overbound the actual one-sided joint travel. This is a research tool concept, not an identified commercial actuator or manufacturing CAD.

The actual FR3 v2.1 visual geometry supplies individual link bounds, plus link8 flange geometry, with kinematics extracted from vendor USD. The pose reference is link7 point [0, 0.121, 0.218] m. The chosen tool orientation places the wrist on positive Y. A rigid held-cable envelope covers 200×16×0.6 mm. This does not establish a stable physical grasp at the nominal 10 mm end setback.

`mount-clearance-001` preserves an initial environment/import failure. `mount-clearance-002` used world-aligned tool bounds reboxed into the wrist frame. `mount-clearance-003` tightened those bounds using the primitive-local extents and their composed transforms; its candidate comparison at 25° retained a minimum 8.379 mm projected gap on negative Y. Positive-Y mounting collided with the cable envelope.

The first rendered review (`mount-review-001`) exposed a visualization bug: `/World/Tool` was a Scope, so its authored root transform did not move the tool. It is preserved and excluded from publication. Defining that root as an Xform corrected the review (`mount-review-002`). The corrected 25° image revealed severe tool occlusion despite adequate mechanical clearance.

The 45° candidate (`mount-clearance-004`, `mount-review-003`) addresses this. It retains a 17.000 mm minimum projected gap in 211 sampled IK poses; the closest pair is the tool neck and the conservative lens box. The positive-Y stand still overlaps the cable envelope and is rejected. The published video uses only the corrected 45° review. All rendered configurations are static; zero physics time elapsed.

## Check definition and margins

The OBB separating-axis test checks three axes from each box and nine pairwise cross products. The maximum positive projection gap is a lower bound on Euclidean clearance. Negative values mean overlapping enclosing boxes, not measured mesh penetration. Lens and cable-routing shapes use enclosing boxes, making a passed separation conservative for those stated shapes.

Sample spacing is at most 2 mm of nominal Cartesian translation. IK is solved within vendor joint limits. These are installation-planning samples, not commands sent to a robot. Full-arm self-collision, the PCB, shared baseplate, desk and unrelated fixtures were not collision-qualified by this camera-hardware study. The standalone tool travel and rigid cable assumptions do not model contact or deformation.

`translation-sweeps.json` additionally encloses the full continuous translation of the nominal fixed-orientation tool and cable for each route segment. The swept-box result also has a 17.000 mm lower bound. Subtracting an assumed **combined** 2 mm assembly allowance leaves 15 mm, against a study design requirement of 5 mm. This allowance is an engineering budget, not measured manufacturer or robot accuracy. The robot links themselves remain sampled IK checks; no continuous full-arm trajectory certification is implied.

## Route and latch access

The held cable first lowers at +80 mm Y, then moves sideways to the insertion line while its leading end is 30 mm ahead of the opening. It advances along X only to a 0.3 mm pre-contact gap. Withdrawal reverses the approach, shifts +80 mm Y, and then lifts +80 mm Z. Lifting directly under the camera is deliberately outside this route. A separate two-pose/15-sample pre-contact latch corridor is reserved; it does not model slider contact, prove latch operability or verify seating.

## Visibility evidence

`mount-visibility-001` compares identical 45° near-entry geometry with and without the complete tool. Offline instance IDs label the actual upper/lower front faces and an appearance-preserving 0.3 mm leading band. No label enters an actor or controller.

- Upper rim: 8,618 baseline pixels; 8,597 with tool, 99.76% retained.
- Lower rim: 548 baseline; 522 with tool, 95.26% retained.
- Leading band: 47,982 in both views, 100% retained.

The lower rim has a small visible area even without the tool. Retention is relative to the same pose and does not establish a fully visible opening, calibrated depth, optical resolution, model accuracy or insertion tolerance. These instance masks use geometric visibility; finite-aperture RGB includes uncalibrated blur and renderer artifacts. Generalization to board rotation, cable curvature, other approach gaps and glare needs a separate dataset.

`mount-visibility-002` retains the corresponding blocked 25° comparison. The page includes its raw full-tool image so the failure is visible.

## Reproduction and next checks

```bash
.venv/bin/python scripts/check_macro_mount.py --profile config/macro-mounted-camera.json --output outputs/mount-clearance-NEW
docker compose run --rm experiment scripts/render_macro_mount.py --report /workspace/outputs/mount-clearance-NEW/report.json --output /workspace/outputs/mount-review-NEW
```

`audit_mount_sweeps.py` and `publish_macro_mount.py` point explicitly to the selected recorded runs; do not overwrite raw evidence when changing candidates. Tests cover axis-aligned separation, touching/overlap, rigid-transform invariance and the distinction between a projection bound and Euclidean distance. The next design gates are manufacturable tool/mount details and stiffness, a continuous full-cell collision check, and macro perception across perturbations. Robot motion remains disabled.

Final checks: 97 tests passed. Desktop (1440 px) and mobile (390 px) browser checks loaded all 24 selected review images with no script errors or horizontal overflow. The 45-degree live preview is advancing all three camera sequences. The rejected 25-degree probe retained zero labelled upper-rim, lower-rim and leading-band pixels with the full tool.
