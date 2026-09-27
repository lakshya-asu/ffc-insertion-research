# Static camera ingress and isolated perception prototype

This experiment predates the Raspberry Pi hardware selection. It uses the generic 8 mm cable and placeholder workcell. It is retained as infrastructure evidence, not as qualified perception on the new Pi scene.

Two fixed 960 × 640 cameras capture authored static poses. No physics time or robot actions are requested. Clean RGB, image hashes and ideal camera calibration are stored in `sensor/`; semantic masks remain in `offline/` for evaluation. Host monotonic timestamps describe render completion, not hardware exposure or synchronized multimodal acquisition.

`CameraGate` enforces image and calibration shape, packet fields, timestamp freshness and sequence ordering. A CPU-only worker runs without network, GPU, simulator mounts or offline labels, receiving read-only sensor and code mounts. A quadratic RGB pixel classifier is fitted offline on 48,000 sampled pixels from 12 development images. Its frozen coefficients and connected-component checks produce candidate masks/endpoints, never robot commands. Source isolation does not imply adversarial containment or a completed real-robot supervisor.

## Evaluation

The selected run is `outputs/m1-fixed-camera-final-002`, with eight new ordinary poses seen by two cameras and four challenge conditions seen by both cameras. The classifier was frozen before final capture.

- Ordinary set: 16/16 mask checks pass IoU ≥ 0.85; mean IoU 0.93256.
- Endpoint gate: 2/13 eligible ordinary views pass maximum error ≤ 3 pixels. Three views have at least one true end outside the image.
- Challenges: only 1/6 positive images passes the mask check. Two empty images yield no false detections. Partial occlusion, ambiguous distractors and low contrast remain failures or rejections.
- Replaying identical sensor bytes with offline labels withheld yields identical predictions and mask bytes on all 24 images, excluding runtime latency. Neither inference run mounted the offline directory.
- A complete reproduction in `outputs/m1-reproduce-001` passes 16/16 ordinary mask checks with mean IoU 0.93207, but only **1/13** endpoint checks. RTX image variation changes a borderline result. This is another reason not to claim the endpoint gate passes.

Earlier colour-threshold and learned-region-selection failures are retained in local outputs. The website shows final-002, not a pooled or selected success rate. Training on old validation cases does not make those cases held-out evidence.

## Consequence

**Motion is disabled.** The detector does not identify contact face, insertion end, depth or tactile state. The Pi hardware asset gate now precedes further task perception development. Existing image ingress and process-isolation software may be reused after new captures and evaluations.

```bash
scripts/run_sensor_milestone.sh your-fresh-run-id
```

This reproduces the historical generic-scene prototype. It does not render the Pi scene. See [the Raspberry Pi CAD milestone](009-raspberry-pi-cad.md) for the selected hardware and next gates.
