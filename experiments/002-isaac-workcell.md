# E002: Isaac workcell construction and smoke test

Hypothesis: a self-contained FR3 USD and a parametrically authored workcell can load, render, and simulate on this RTX 5080 host with bounded resting cable motion. This does not test insertion success.

Before execution: use isolated Python 3.12 / Isaac Sim 6.1.0.0; fixed seed 260927; 240 Hz physics. Include a physical ribbon surrogate, connector slot and driven latch, deployable pinch mechanism, passive nest, and three camera views. Keep initial cable on desk and all grasp attachments disabled.

Acceptance: USD composes without unresolved assets; seven named FR3 arm joints exist; cable dimensions and mass match configuration; adjacent cable links share coincident joint anchors; connector aperture is wider/thicker than cable; at least 240 physics steps produce finite robot/cable state without cable loss through desk; RGB captures are nonempty. Inspect whole-cell and close views. Validate initial slot geometry numerically, not by its appearance in a distant image.

Known limits: FR3 v2.1 asset instead of earlier Menagerie FR3 variant; no calibrated cable laminate mechanics, vacuum flow, contact resistance, damage model, or verified autonomous assembly policy. A resting smoke test is not a pickup or insertion demonstration. Track tool joint exercise separately from assembly success.

Results: pending execution.
