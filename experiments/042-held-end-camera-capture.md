# 042 — Held-end camera capture

The blocked capture from 041 ran successfully after Docker access was restored. Source: mounted lift fr3-flex-008, native recorded-state replay. Capture: outputs/held-end-capture-001. No physics rerun, new control or GPU learned inference is claimed.

Command: `COMPOSE_IGNORE_ORPHANS=1 docker compose run -d --name ffc-held-end-capture-001 experiment scripts/capture_held_end_inspection.py --stage /workspace/docs/library/fr3-flex-008/replay.usdz --config /workspace/config/held-end-inspection-v1.json --output /workspace/outputs/held-end-capture-001`. Use a fresh name/output on repetition.

Seven frames cover 0.05, 0.5, 1, 1.5, 2, 2.5 and 3 seconds. Every raw frame was visually inspected. Contact fingers/free edge are visible; mid-stroke appears sharper than either end. The upper terminal clips near the final pose and the tool reference is cropped. Segmented ribbon rendering leaves visible seams. This is not a photorealism or edge-localization qualification.

The Basler/Kowa reference uses a fixed 45-degree view, 1224×1024 render, synthetic K/optical transform, idealized zero distortion and finite aperture. Physical lens behavior, exposure, camera mounting clearance and contact physics remain separately unqualified.

The review generator verifies image SHA256, calibration identity, dimensions and ordered observation ingress. Its RGB-only heuristic always leaves leading edge and slip unverified and motion disabled. Replay-clock ingestion simulates immediate offline delivery; it does not test real latency. Seven adjacent samples are one episode, not a train/test split.

[Public review](../docs/held-end-review/capture-001/index.html) includes raw images, masks, measurements, video and USDZ. Video is a seven-sample montage held one second per frame, not continuous robot motion. The USDZ retains the inspection camera and disabled rigid bodies; it reopened with USD. The original exported camera scene was rendered in Isaac.

Validation: 178 CPU tests passed. The previous Docker restriction and bad colour-baseline evidence are retained in 041 and the historical review. Next: center/focus the camera on a repeatable held-end inspection pose, then annotate and independently evaluate the leading edge before enabling alignment.
