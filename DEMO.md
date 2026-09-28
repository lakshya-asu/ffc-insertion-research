# Presentation demo

Open **https://lakshya-asu.github.io/ffc-insertion-research/demo/**. It has live prediction, whole-cell view, recorded fallback and a five-minute presenter script. The page opens in a normal browser; the presenter does not need Tailscale or a local install. The live tunnel depends on this lab machine; the hosted recording does not.

Download [the fallback video](https://lakshya-asu.github.io/ffc-insertion-research/demo/live-perception.mp4) before presenting. It is approximately 40 seconds of actual ROS inference on the stationary simulated scene, with capture cadence retained. It has no audio; the page provides the spoken walkthrough. For diverse poses and misses, use the separate [60-scene review](https://lakshya-asu.github.io/ffc-insertion-research/live/#macro-model).

## What this demonstrates

Native simulated macro RGB and calibration enter ROS, pass through classical preprocessing, and reach the frozen mounted-camera DINOv3 model. The runtime has no USD or offline annotation mount. The model publishes acquisition-stamped masks only after checking the fixed rig, native/processed calibration, model identity and post-inference age. The human viewer joins each mask to its exact source observation.

Show the camera RGB and overlay side by side. Explain the entrance rims, cable leading band and slider classes. The deadline is 500 ms; the renderer is roughly 1.2 fps. Saved/expired labels between updates are expected. They are useful evidence that the display does not disguise a stale frame as a current control observation.

This is not an insertion demonstration. The cell is stationary. No actuator endpoint is launched. Confidence is uncalibrated; metric pose, deformable contact, grasping and force/tactile feedback are unfinished. Do not describe a successful segmentation as a successful assembly.

## Before the meeting

On the existing lab host:

```bash
cd /home/flux/ffc-insertion-research
bash scripts/start_demo.sh
# Allow the GPU model and Isaac camera to start, then:
python3 scripts/check_demo.py
```

The Docker image is built with `docker build -f Dockerfile.ros2 -t ffc-ros2:jazzy-dinov3 .`. It requires the existing foundation image and locally approved model downloads. The repository does not redistribute gated weights. Startup checks for the reviewed scene and weights, then starts preprocessing, inference and the read-only viewer. It preserves an already-running renderer. The static camera/viewer are configured for 12-hour sessions; rerun startup near the presentation if needed.

Confirm the public page can connect and that the model hash begins `8668408a864e`. Check live feature age changes, switch to whole cell, play the fallback, and download it. If the tunnel has expired, restart the existing tunnel service and update `docs/live/connection.json` with its new public origin before publishing. Do not enable a general filesystem server or robot controls to fix the connection.

For local access use `http://127.0.0.1:8766/demo/`. A browser clock error affects the displayed client-side age; the ROS admission deadline uses the host acquisition clock.

## Failure demonstration

Experiment 024 records actual DDS tests: missing TF, stale input, wrong profile, wrong native calibration, truncated edges, self-consistent but wrong processed intrinsics, fresh recovery, dropout and changed camera TF. A separate delayed-model test injected 600 ms inside the test harness and emitted zero features. The production node has no delay-injection parameter.

The web page is read-only. Its source switches change the presentation, not the robot or sensor state. Use the recorded test outcomes when discussing failure behavior; do not claim clicking a page button injects a live fault.

## Recovery and shutdown

If the camera TF changes, inference intentionally latches a fault. Inspect the mounting/calibration change before restarting `inference`; silently accepting restored TF would defeat that check. Other rejected frames recover when a fresh, valid observation arrives.

Stop the demo’s inference and viewer with:

```bash
docker compose -f compose.yaml -f compose.ros2.yaml -f compose.inference.yaml stop inference feature_view
```

To yield the renderer, create `outputs/lab-live/stop-preview` and let it stop cleanly. The hosted video remains available independently of the live feed.
