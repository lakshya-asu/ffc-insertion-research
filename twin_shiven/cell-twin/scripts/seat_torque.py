"""Calibrate contact friction/stiffness against the real seat torque (~3 Nm rise on joint 2).
Runs a zero-error straight push for several friction values and reports the peak joint-2 rise and contact force."""
import os, sys, numpy as np, importlib
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent; sys.path.insert(0, str(ROOT))
seated = np.load(ROOT / "inputs/nuc/episodes/objects/09710851-e26c-43ef-b7ed-0a684c28e4a4/frames.npz")["observation.state"][-1]
for mu in (0.4, 0.8, 1.2, 1.6):
    os.environ["TWIN_FRICTION"] = str(mu)
    import cell_twin.scene as S; importlib.reload(S)
    import cell_twin.bench as B; importlib.reload(B)
    import cell_twin.controllers as C; importlib.reload(C)
    b = B.Bench(seated, B.PickError(0.0)); base = np.mean([b.torques() for _ in range(5)], axis=0)
    peak = np.zeros(6); fmax = 0.0; seat_tick = None
    for t in range(60):
        b.step_tool([0, 0, C.APPROACH_STEP], [0, 0, 0]); tau = b.torques()
        peak = np.maximum(peak, np.abs(tau - base)); fmax = max(fmax, b.plug_contact_force())
        if seat_tick is None and b.seated(): seat_tick = t
        if seat_tick is not None and t > seat_tick + 8: break
    print(f"mu {mu}: seated at tick {seat_tick}; peak torque rise per joint (Nm) {peak.round(2)}; j2 {peak[1]:.2f}; peak contact force {fmax:.1f} N", flush=True)
