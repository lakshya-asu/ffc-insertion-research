"""Capture-region sweep for a controller: P(verified seat under the force limit) per (lateral, yaw) start cell,
with Wilson intervals, physics randomized inside each cell. Writes JSON and an SVG heatmap to out/.

usage: capture_region.py [--n 200] [--estimator good|poor|none] [--controller expert|push] [--level L1] [--procs 8]
"""
import argparse
import json
import math
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffc_twin.env import TwinEnv, EstimatorConfig  # noqa: E402
from ffc_twin.expert import Expert  # noqa: E402
from ffc_twin.spec import make_spec  # noqa: E402
from ffc_twin.stats import wilson  # noqa: E402

LAT = [-1.5, -1.0, -0.6, -0.3, 0.0, 0.3, 0.6, 1.0, 1.5]
YAW = [-6, -4, -2, 0, 2, 4, 6]


def _cell(args):
    lat_mm, yaw_deg, n, est_name, controller, level, seed0, board, grip, dof, model = args
    env = TwinEnv(spec=make_spec(board, grip, dof), estimator=EstimatorConfig.named(est_name), log=False)
    bc = None
    if controller == "bc":
        from ffc_twin.policy import BCPolicy

        bc = BCPolicy(model)
    ok = 0; peaks = []; retries = []
    for i in range(n):
        seed = seed0 + i
        obs, _ = env.reset(seed=seed, level=level, start=dict(lateral=lat_mm * 1e-3, yaw=math.radians(yaw_deg)))
        ex = Expert(env.spec)
        if bc is not None:
            bc.reset(); ex = bc
        for _ in range(150):
            a = ex.act(obs) if controller in ("expert", "bc") else np.array([1, 0, 0, 0, 0, 0.0])
            obs, r, term, trunc, info = env.step(a)
            if term or trunc:
                break
        t = env.truth()
        seated = t["seated"] and env.peak_force < env.spec.success.peak_force_limit.value
        ok += int(seated); peaks.append(env.peak_force); retries.append(ex.retries)
    p, lo, hi = wilson(ok, n)
    return dict(lat_mm=lat_mm, yaw_deg=yaw_deg, n=n, ok=ok, p=p, lo=lo, hi=hi, peak_force_median=float(np.median(peaks)), retries_mean=float(np.mean(retries)))


def heatmap_svg(cells, title, sub):
    cw, ch, x0, y0 = 46, 36, 70, 40
    w = x0 + cw * len(LAT) + 20; h = y0 + ch * len(YAW) + 60
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" font-family="-apple-system,Inter,sans-serif">',
         f'<text x="{x0}" y="18" font-size="13" font-weight="600">{title}</text>', f'<text x="{x0}" y="32" font-size="11" fill="#666">{sub}</text>']
    idx = {(c["lat_mm"], c["yaw_deg"]): c for c in cells}
    for j, yw in enumerate(YAW):
        for i, la in enumerate(LAT):
            c = idx[(la, yw)]; p = c["p"]
            col = f'rgba(15,138,95,{0.12 + 0.78*p:.2f})' if p > 0 else 'rgba(185,28,28,0.10)'
            s.append(f'<rect x="{x0+i*cw}" y="{y0+j*ch}" width="{cw-2}" height="{ch-2}" rx="3" fill="{col}"/>')
            s.append(f'<text x="{x0+i*cw+cw/2-1}" y="{y0+j*ch+ch/2+4}" text-anchor="middle" font-size="10">{int(round(p*100))}</text>')
        s.append(f'<text x="{x0-8}" y="{y0+j*ch+ch/2+4}" text-anchor="end" font-size="11" fill="#666">{yw:+d}°</text>')
    for i, la in enumerate(LAT):
        s.append(f'<text x="{x0+i*cw+cw/2-1}" y="{y0+len(YAW)*ch+14}" text-anchor="middle" font-size="10" fill="#666">{la:+.1f}</text>')
    s.append(f'<text x="{x0+cw*len(LAT)/2}" y="{y0+len(YAW)*ch+34}" text-anchor="middle" font-size="11">lateral start error, mm</text>')
    s.append(f'<text x="14" y="{y0+ch*len(YAW)/2}" text-anchor="middle" font-size="11" transform="rotate(-90,14,{y0+ch*len(YAW)/2})">yaw start error</text></svg>')
    return "".join(s)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=200); ap.add_argument("--estimator", default="good"); ap.add_argument("--controller", default="expert")
    ap.add_argument("--level", default="L1"); ap.add_argument("--procs", type=int, default=8); ap.add_argument("--tag", default="")
    ap.add_argument("--board", default="pi4", choices=["pi4", "zero"]); ap.add_argument("--grip", default="tape", choices=["tape", "body"]); ap.add_argument("--dof", type=int, default=6, choices=[4, 6])
    ap.add_argument("--model", default="", help="model directory for --controller bc")
    a = ap.parse_args()
    if a.model:
        a.tag = a.tag + "_" + Path(a.model).name
    jobs = [(la, yw, a.n, a.estimator, a.controller, a.level, 100000 * (i + 1), a.board, a.grip, a.dof, a.model) for i, (la, yw) in enumerate((la, yw) for yw in YAW for la in LAT)]
    with Pool(a.procs) as pool:
        cells = pool.map(_cell, jobs)
    name = f"region_{a.board}_{a.grip}_{a.dof}dof_{a.controller}_{a.estimator}_{a.level}_n{a.n}{a.tag}"
    out = Path(__file__).resolve().parents[1] / "out"; out.mkdir(exist_ok=True)
    (out / f"{name}.json").write_text(json.dumps(dict(board=a.board, grip=a.grip, dof=a.dof, controller=a.controller, estimator=a.estimator, level=a.level, n_per_cell=a.n, cells=cells), indent=1))
    (out / f"{name}.svg").write_text(heatmap_svg(cells, f"{a.board}, {a.grip} grip, {a.dof} axes, {a.controller}, estimator {a.estimator}, level {a.level}", f"{a.n} runs per cell; number = percent seated under the force limit"))
    grid = np.array([[next(c["p"] for c in cells if c["lat_mm"] == la and c["yaw_deg"] == yw) for la in LAT] for yw in YAW])
    print(name); print(np.round(grid * 100).astype(int)); print("mean", round(float(grid.mean()) * 100, 1), "percent")
