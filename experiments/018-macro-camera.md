# A macro observation camera for the Zero connector

Selected simulation reference: Basler ace 2 a2A2448-75ucBAS + Kowa LM35JC10M, without extension rings. Configuration: `config/macro-basler-kowa.json`. Both manufacturer pages are recorded there. No hardware was ordered.

The Basler default 2448×2048 active region at 2.74 µm pitch is paired with the lens's published 0.38× magnification at 100 mm minimum working distance. Calculated normal-plane field: 17.651×14.767 mm; sampling 7.211 µm/pixel. This does not establish feature resolution or dimensional accuracy. We prefer the conventional lens for this first integration because its perspective projection maps to ordinary camera calibration. A telecentric alternative needs a separate projection/calibration contract and a larger mechanical envelope.

## Placement and evidence

`outputs/macro-setup-001` preserves yaw 35°, elevation 25°, target 1 mm ahead of the opening. `outputs/macro-setup-002` selects yaw 0°, elevation 25°, target 0.15 mm ahead. Both contain native images at cable gaps 6, 3, 1 and 0.3 mm, with ideal and finite-aperture optics, four placement views, report and composed USD. Each capture asserts zero elapsed physics time. All edits are in a session layer; the source scene is unchanged. The selected scene is `outputs/macro-setup-002/macro-workcell.usda`.

Manual inspection found the diagonal view puts opposite ends of the aperture at different focus distances. The selected frontal view keeps those ends at similar distances. At 3 mm, the cable remains defocused; at 0.3 mm the leading edge is clearer while the distant jaw is intentionally outside focus. This is visual inspection, not a detector pass rate or calibrated sharpness metric. Camera/lens volumes are dimensioned primitives: 43×49 mm lens and 29×29×48.1 mm camera. No bracket, connector intrusion drawing or cable routing has been qualified. The body's position is a conservative envelope, not exact mounting geometry.

## Optical limits

Thin-lens object distance u = 35(1+1/0.38) = 127.105 mm; image distance v = 35(1+0.38) = 48.3 mm. The virtual principal plane is inferred 27.105 mm behind the lens front. USD uses v as the perspective projection parameter and physical sensor dimensions, yielding the specified magnification at u. This is a projection equivalent, not a calibrated optical prescription.

Physical f/8 gives an equivalent renderer f-number 11.04 to preserve the assumed 35/8 mm pupil diameter. Approximate total DOF is 0.419 mm for a one-pixel circle of confusion, ignoring diffraction. With unit pupil magnification, the estimated diffraction Airy diameter at 550 nm is about 5.4 pixels at this magnification. The renderer does not simulate that diffraction or the manufacturer's MTF. It also shows some defocus edge artifacts; do not treat these images as a physical optical qualification. Small f-numbers and stopping down both have costs. No universal “best setting” has been demonstrated.

30 fps and 500 µs are starting hardware targets. At 1 mm/s, that exposure produces about 0.07 pixel motion in the normal object plane; at 10 mm/s, about 0.69 pixel. The Bayer8 payload would be about 150 MB/s, before protocol overhead; RGB8 would be about 451 MB/s. Pixel format and transport must be selected and measured in the eventual driver. Exposure does not currently control simulated radiometry. Use diffuse controlled lighting, then test glare and clipped highlights; lighting hardware is not selected here.

## Perception and ROS boundary

The existing fixed 4K crop model cannot be silently applied to this 2448×2048 profile. `runtime.model_compatible=false`; `motion_permitted=false`. Reserved topics are `/cell/cameras/macro/image_raw` and `/cell/cameras/macro/camera_info`, optical frame `macro_optical_frame`. No native macro ROS publisher or model has been qualified yet. Classical rectification must transform CameraInfo along with any crop/resize. A new native-resolution development split and independent frozen-model test are required.

Preview images and the 16-second review video are published under `docs/live/macro`. They are static poses, never a successful insertion demonstration. The existing aperture is an engineered substitute with assumed dimensions, and the cable and fingers are rigid visibility proxies. The human preview may use the USD scene; no runtime controller reads it.

## Reproduction

```bash
docker compose run --rm experiment scripts/render_macro_setup.py --output /workspace/outputs/macro-setup-NEW
.venv/bin/pytest -q tests/test_macro_camera.py
```

The projection test catches the USD millimetre/scene-unit scaling error and checks field of view against the selected sensor and magnification. Full local suite: 94 tests passed after this addition.

## Preview reload correction

The first saved scene included Replicator's process-local `/Render` graphs, which caused a duplicate graph error when reopening for preview. The failed service log is preserved in journald. `outputs/macro-preview-scene-002/macro-workcell.usda` is a separate cleaned export with `/Render`, `/Replicator` and `/Orchestrator` removed; geometry and cameras are unchanged. Future capture exports now strip these transient graphs. The original two capture directories remain intact.

A second reload exposed a remaining `/Orchestrator` graph and stalled capture scheduling. The intermediate cleaned scene `macro-preview-scene-001` is preserved; `macro-preview-scene-002` strips all three transient roots. The preview now refuses to publish a new timestamp if Replicator remains in STEPPING after its synchronous capture call. This prevents a failed capture being presented as a fresh image.
