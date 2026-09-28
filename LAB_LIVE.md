# Live research view

The user wants to follow the cell, the current work and the engineering decisions on one page. Keep this view current during future experiment work.

- Stable entry: https://lakshya-asu.github.io/ffc-insertion-research/live/
- The current temporary origin is recorded in `docs/live/connection.json`.
- Local origin: `http://127.0.0.1:8766/live/`.
- Service: `ffc-lab-live.service` in the user's systemd session. Only the `docs/` website, curated status and three camera snapshots are served. Directory listing and write/control requests are rejected.
- Tunnel: `ffc-lab-tunnel.service`. Uses the verified Cloudflare release recorded in ignored `outputs/lab-live/tunnel-install.json`. This is a temporary Quick Tunnel; its URL can change on restart. Update `connection.json` and the GitHub site if it changes.
- Renderer: `ffc-lab-preview.service`. A stationary Isaac preview with zero physics advancement. It yields fresh views until its configured duration ends. The browser shows frame age and explicitly labels stale frames as saved images.

## Publish meaningful updates

```bash
python3 scripts/publish_lab_status.py \
  --phase capture \
  --title 'Short description of the current work' \
  --detail 'What changed, the evidence, and any failure that affects interpretation.' \
  --next 'The next concrete check.'
```

Publish at major transitions, meaningful findings, failures and completion. Summarize design rationale and tradeoffs; do not export terminal logs, credentials, personal files or private deliberation. The live service reads the current training history from the explicitly configured run in `scripts/serve_lab_live.py`. Update that run path when starting a new model; preserve failed runs and identify them in the work log.

Before dataset capture, stop the separate preview cleanly so both processes do not overwrite the same human-view feed:

```bash
touch outputs/lab-live/stop-preview
```

Wait for that service to exit. Then pass `--live-feed /workspace/outputs/lab-live/feed` to `capture_pi_perception.py`. Its optional third camera is for human observation only; it is not part of the training dataset. The desk and board camera images remain the model inputs.

After capture, remove that owned stop flag and start the preview again if useful. The preview can share the tested GPU with this small network, but monitor memory; the dashboard must retain and timestamp its last image when the renderer is paused.

```bash
rm outputs/lab-live/stop-preview
systemd-run --user --unit=ffc-lab-preview --working-directory="$PWD" \
  /usr/bin/docker compose run --rm experiment scripts/stream_pi_scene.py --seconds 43200
```

The camera, status service and tunnel are independent. To stop the public connection and service:

```bash
systemctl --user stop ffc-lab-tunnel ffc-lab-live
```

The notes field is browser-local storage. It is not an agent inbox. The user copies notes into the chat when they want a response. Do not imply that browser notes have been submitted.

## Observation boundary

This page observes the simulated workcell. It has no actuator endpoint. Actor/critic inputs, rewards and stage transitions must use deployable measurements in future work. Simulator geometry and labels are permitted for offline supervision and evaluation, not privileged runtime control.

## Current scene selection

The current preview is the separate Zero 2 W side-entry scene. Restart it with the usual command plus:

```bash
--stage /workspace/outputs/pi-zero-review-002/pi-zero-workcell.usda \
--layout /workspace/config/pi-zero-task.json --board-camera entrance
```

With no extra arguments, the renderer still selects the Pi 4 / Arducam reference. The page's saved Zero review is `/live/#zero`; its virtual macro is for human geometry inspection and is not a deployed observation camera.

## Macro preview

The current macro preview uses the composed static scene from experiment 018:

```bash
systemd-run --user --unit=ffc-lab-preview --working-directory="$PWD" \
  /usr/bin/docker compose run --rm experiment scripts/stream_macro_scene.py \
  --stage /workspace/outputs/mount-review-003/mounted-workcell.usda --seconds 43200
```

The Pi close-up now shows a half-resolution Basler/Kowa reference, with finite-aperture sensitivity blur. Cable view shows the camera placement envelopes. This human preview is not a ROS camera publisher or perception input. The same stop-preview flag cleanly yields the GPU for later experiments. Saved native images and comparison video: `/live/#macro`.

The active mounting revision is experiment 019: full tool, braced opposite-side stand and a 45-degree camera view. `/live/#mount` records clearance and visibility evidence. The older `/live/#macro` is retained as the earlier optics study.

## Native ROS camera milestone

Experiment 020 adds native RGB publication to the mounted preview. Start it with `docker compose -f compose.yaml -f compose.ros2.yaml run --rm experiment` and the receiver with `docker compose -f compose.yaml -f compose.ros2.yaml up -d perception`. This supersedes the non-ROS preview description above. The human feed remains read-only. ROS observations use wall-clock acquisition timestamps for this static scene, and motion stays disabled. Native rendering currently runs about 1.2 fps; the 500 ms watchdog therefore correctly reports stale intervals. See `/live/#ros-camera`.

## Live learned-perception demo

Experiment 024 connects the frozen mounted model through `compose.inference.yaml`. The presentation entry is `/demo/`; `/api/perception` serves an atomic matched RGB/prediction review snapshot with acquisition identity. The human viewer is read-only. `bash scripts/start_demo.sh` starts the reviewed services; `python3 scripts/check_demo.py` checks a recent feature frame and the recorded fallback. The actual recording and presenter script are linked from `DEMO.md`. Static scene and motion-disabled limits remain in force.
