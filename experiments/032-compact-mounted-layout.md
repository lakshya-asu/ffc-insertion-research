# 032 — Compact tool on the FR3

The compact PGEA and custom fingers now replace the old mechanism in a separate
stationary candidate scene. The robot remains at the existing workcell pose.
This is a visual mounting and CAD-clearance study, not a physically commissioned
tool or an insertion demonstration.

## Adapter concept

The proposed bracket has a 63 mm outside-diameter, 6 mm thick flange ring and an
18 mm wide, 6 mm thick side arm. Two 3.4 mm clearance holes line up with the
PGEA's documented M3 holes on its lower broad face, 10 mm apart. Nominal M3 × 8
screws would engage 2 mm of the drawing's 4 mm thread depth; these fasteners are
not yet modeled or selected for release.

The ring uses four 6.6 mm clearance holes on a 50 mm pitch circle, at a 45-degree
offset from the cardinal axes, based on Figure 6.11 in the
[FR3 product manual](https://www.franka.de/hubfs/Product%20Manual%20Franka%20Research%203_R02210_1.5_EN-1.pdf).
The exact angular registration to the factory USD frame and locating fit still
need qualification. The 32 mm center opening does not by itself prove wrist
connector access. No flange screws, locating features or strain relief are
claimed as completed hardware.

The gripper runs along the flange tool axis, and its jaw-closing axis is vertical
in this parked pose. Finger fronts sit 107 mm beyond the flange plane. The
supplier side-exit cable geometry remains present. The nozzle and vacuum tubing
are deliberately absent until their geometry and routing are designed.

## Checks and limits

- The bracket is one valid connected CAD solid. Its intersection volume with
  the fixed supplier gripper geometry is zero.
- Eight sampled base openings from 1 to 11 mm, including 1.14 and 1.30 mm, have
  zero bracket-to-moving-supplier-component intersection volume.
- The complete mounted tool's axis-aligned bounding box is separated from the
  macro-camera envelope by at least 13.858 mm in this pose. The corresponding
  lower bound to the board is 86.650 mm.
- Bounds include the supplier cable tail and the adapter. They are conservative
  geometric lower bounds, not measured operating clearances. They do not cover
  a trajectory, camera visibility, robot self-collision or a future nozzle.
- The old `/World/Tool` is inactive in this candidate layer. The standalone
  dimensional coupon is also inactive. The actual scene's static PCB and cable
  remain in place.

No dynamics were added. The compact assembly is visually parented to link8;
rigid bodies, inertia, drives, force feedback and grasp contact are still
required. Original workcell and perception assets are untouched.

## Evidence and reproduction

CAD and stage: `outputs/compact-mount-001`. Isaac views and video:
`outputs/compact-mount-render-001`. The six-second video contains 72 frames at
12 fps, with a stationary robot and prescribed empty-jaw opening/closing.
Physics time remains zero. This video is separate from the earlier PQ12
mounted-dynamics evidence.

```bash
/tmp/ffc-tool-cad-env/bin/python design/compact-tool/build_mount.py \
  --source outputs/compact-gripper-sourcing-002/pgea-O-S.STEP \
  --fingers outputs/compact-fingers-002 \
  --base-scene outputs/mount-review-003/mounted-workcell.usda \
  --output outputs/compact-mount-NEW

docker compose run --rm experiment design/compact-tool/review_mount_isaac.py \
  --candidate /workspace/outputs/compact-mount-NEW \
  --output /workspace/outputs/compact-mount-render-NEW
```

Use fresh directories and stop the other renderer first. For desktop viewing,
use the verified X11/display-device compose configuration and add `--gui`.
The review panel switches between the whole cell, mounted tool and bracket.
The GUI is a geometry viewer, not a commissioned physics scene.

## Next

Resolve adapter locating features, fasteners and stiffness together with the
finger material and pad attachment. Then add a fixed vacuum nozzle and its
pressure-sensing route, check desk/fixture access, and repeat clearance and
entrance-visibility studies across candidate approach poses. Sensor-driven
approach motion follows those geometry and mechanics gates.
