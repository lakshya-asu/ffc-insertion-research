# Pi Zero 2 W: sideways ribbon insertion

The user selected the Zero family as a second ribbon-insertion example. The implemented asset is specifically **Zero 2 W**, separate from the Pi 4 task and from the later phone-style press-on task.

## Asset and evidence

The board is the named `Raspberry Pi Zero 2 W` assembly extracted from [Doruk Kumkumoglu's Optocam Zero V1.1 STEP](https://github.com/dorukkumkumoglu/optocamzero). `config/pi-zero-sources.json` pins the upstream commit, bytes and SHA-256. Extracting the named subtree removes the surrounding camera enclosure and resets the outer assembly placement while retaining internal component transforms. The import contains 94 leaf components and 478,024 triangles. It includes passives, through holes, ports, an SD card and a component named `54548-2271, 22 Pin FPC Connector`. This is community CAD, not a certified Raspberry Pi BOM or connector tolerance model.

The first inspected alternative, AchimPieters/KiCad-Templates Zero W STEP, contained only a PCB and header. It was rejected as insufficient for connector perception and remains local as research evidence.

The [official Zero 2 W mechanical drawing](https://datasheets.raspberrypi.com/rpizero2/raspberry-pi-zero-2-w-mechanical-drawing.pdf) supports the 65 × 30 mm outline and 58 × 23 mm mounting-hole pattern. The audit checks the imported PCB bounds and pairwise distances between four 2.7 mm cylindrical holes. All pass. The source PCB thickness is 1.6 mm; the outline drawing does not independently certify it.

The CAD assembly names a Molex 54548-2271. The [Molex 54548 family](https://www.molex.com/en-us/products/series-chart/54548) uses right-angle bottom-contact ZIF connectors. Its slide latch remains unarticulated here. The model's nominal front envelope is used only to place static review probes; it is not an observed mouth pose, safe insertion trajectory or verified latch-open state.

## Cable choice

We use the official Standard–Mini 200 mm camera-cable family: 22 contacts at 0.5 mm pitch at the Zero, 15 at 1 mm at the camera. [PCN 36](https://pip-assets.raspberrypi.com/categories/1267-pcn/documents/RP-009201-PC-1-15%20to%2022%20pin%20camera%20cables%20Rev%202.pdf) documents revision two, which is 11.5 mm wide along most of its length and expands to 16 mm near the 15-pin end.

The visual mesh has this asymmetric outline, individual contact pads and stiffeners. The 27 mm wide-end section, 2 mm taper length, 0.14 mm body thickness, 0.30 mm header thickness, 4 mm exposed contacts and 6 mm stiffener length are **unmeasured modeling assumptions**. The mini contacts face the board; the far-end opposite face is also explicitly unverified. Black body, blue stiffeners, contact finish and omitted markings require real-image calibration. Internal electrical routing is not modeled. This is not a mechanically qualified laminate or a substitute for the actual cable drawing/specimen.

The newer PCN 48 concerns separate 150 mm/38 mm Zero camera-cable variants; it is not used to define this 200 mm revision-two cable.

## Scene and sensing

Board underside is 30 mm above the desk on four visual supports. The connector faces +X; insertion would travel −X, parallel to the PCB and perpendicular to its edge. Three independent situations are rendered: loose cable, a nominal 15 mm pre-entry gap, and a nominal 3 mm gap. The latter two are flat authored cable poses with jaw-envelope blocks. No physics steps, grasp, latch movement, seating or electrical result is claimed.

Three native 3840 × 2160 views use the B0498 / IMX585 16 mm reference: overhead, nearly axial entrance, and offset entrance. A fourth virtual macro is explicitly for human CAD inspection; its short working distance is not claimed as a real Arducam capability. All optics are ideal projections with DOF disabled. The macro must not silently enter a training dataset as a deployable sensor.

The initial run `outputs/pi-zero-review-001` clipped the loose cable’s narrow end in the overhead frame. It is preserved as placement evidence. Revision two recenters the overhead view; an independent USD projection test checks that both cable ends lie inside the image with margin.

Run `outputs/pi-zero-review-002` saves 12 RGB frames, the new scene USD, camera settings, provenance and the geometry audit. All public WebP copies match source pixels exactly. A 12-second montage shows the three static situations. Live preview uses this scene, the overhead view and the entrance view at 960 × 540; the whole-cell camera remains a human observer. The old Pi 4 scene and model evaluation remain intact. No new perception model has been trained on these views.

## Reproduction

From the repository root, fetch and verify the locked sources, then use the separate CAD environment:

```bash
python3 scripts/fetch_raspberry_pi_cad.py --manifest config/pi-zero-sources.json --destination third_party/raspberry_pi/zero
.venv-cad/bin/python scripts/convert_step_to_usd.py third_party/raspberry_pi/zero/optocam.step third_party/raspberry_pi/zero/zero2w.usdc --assembly-name 'Raspberry Pi Zero 2 W'
.venv-cad/bin/python scripts/audit_pi_zero.py
```

Stop the live preview with the documented `LAB_LIVE.md` flag before capture. Render to a fresh directory:

```bash
docker compose run --rm experiment scripts/render_pi_zero.py --output /workspace/outputs/pi-zero-review-NEW
.venv/bin/python scripts/publish_pi_zero_review.py outputs/pi-zero-review-NEW docs/live/zero-study
```

The base scene remains the locally imported/refined Pi 4 workcell, composed as a USD sublayer with the Pi 4, its supports and original cable deactivated. The separate Zero asset and cable occupy their own namespaces. Reproduce the base scene using experiments 009–010 first on a new machine.

The next gate is to verify physical connector/latch geometry and cable end dimensions, then measure whether deployed optics resolve the entrance and tip under tool occlusion. Simulator geometry remains offline asset/supervision data; it is excluded from runtime perception, policy, critic, reward and transition inputs.

## Attribution

Optocam Zero: Doruk Kumkumoglu, CC BY-SA 4.0. Changes: extracted board subtree, USD tessellation, workcell placement, lighting, and additional cable/camera review geometry. The Zero review images and montage are shared under CC BY-SA 4.0, with `docs/live/zero-study/ATTRIBUTION.txt`. Source CAD is retained locally in ignored `third_party/`.
