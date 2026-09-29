# Project working agreements

- Read `LAB_LIVE.md` before experiment work. The user wants to follow the current cell view, experiment progress, decisions and failures. Publish concise work updates at meaningful transitions and keep the live page's current-run pointer accurate.
- New perception and control must use deployable camera, tactile and measured robot/tool signals. Simulator truth belongs only in offline supervised labels and evaluation. Do not reconnect historical privileged-state control to the new Pi task.
- The current Pi scene is for static synthetic perception development. It does not qualify real-camera perception, insertion contact, latch mechanics or cable deformation. The user explicitly requests a full simulation proof of concept before real hardware. Real-camera tests are deferred, not a blocker. Keep robot motion disabled until the relevant simulation mechanics and sensor-based control gates exist and pass; do not label simulated validation as physical validation.
- Preserve run directories and failed evidence. Use fresh output directories; freeze a model before evaluating an independent test set.
- Do not spawn sub-agents unless the user explicitly asks for delegation or parallel agent work.

- Keep `research/learning/PLAN.md` and `SEVEN_STEPS.md` current at milestone boundaries. Archive motion with original videos, native physics-disabled USD replay, source/config/asset identities and rerun instructions. Label playback separately from physics reruns and preserve failed evidence.
