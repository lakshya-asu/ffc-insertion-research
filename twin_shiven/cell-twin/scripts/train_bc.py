"""Behaviour cloning of the expert's clean actions (spec B5).

usage: python scripts/train_bc.py out/episodes/<run> [more runs] [--out out/models/bc_v1.pt] [--epochs 60]

L1 loss on the clean (noise-free) expert action given the observation window the noisy rollout produced (DART).
20 percent of episodes held out (split by episode), early stop on held-out loss (patience 6 epochs), best epoch kept.
Writes the model and <out>.json with per-epoch train and held-out loss.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402
import torch  # noqa: E402

from cell_twin import datasets as D  # noqa: E402
from cell_twin import models as M  # noqa: E402


def batches(ds, sel, bs, rng=None):
    order = rng.permutation(sel) if rng is not None else sel
    rows = torch.from_numpy(ds.rows)
    for i in range(0, len(order), bs):
        j = order[i:i + bs]
        x = rows[torch.from_numpy(ds.win_idx[j])].reshape(len(j), -1)
        x = torch.cat([x, torch.from_numpy(ds.tau_th[j])[:, None]], 1)
        yield x, j


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out", default=str(ROOT / "out" / "models" / "bc_v1.pt"))
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--patience", type=int, default=6)
    ap.add_argument("--bs", type=int, default=1024)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--threads", type=int, default=4)
    a = ap.parse_args()
    torch.set_num_threads(a.threads); torch.manual_seed(0)
    t0 = time.time()
    eps = D.list_episodes(a.runs)
    ds = D.load(eps, "bc")
    tr_ep, ho_ep = D.split_episodes(len(eps))
    tr = np.flatnonzero(np.isin(ds.episode, tr_ep)); ho = np.flatnonzero(np.isin(ds.episode, ho_ep))
    mean, std = D.row_stats(ds.rows)
    model = M.TorqueMLP(); M.set_norm(model, mean, std)
    opt = torch.optim.Adam(model.parameters(), lr=a.lr)
    Y = torch.from_numpy(ds.action).clamp(-1, 1)
    rng = np.random.default_rng(0)
    hist = []; best = (np.inf, None, -1); bad = 0
    for ep in range(a.epochs):
        model.train(); tl = []
        for x, j in batches(ds, tr, a.bs, rng):
            loss = (model(x) - Y[j]).abs().mean()
            opt.zero_grad(); loss.backward(); opt.step(); tl.append(loss.item() * len(j))
        model.eval(); hl = []
        with torch.no_grad():
            for x, j in batches(ds, ho, 8192):
                hl.append((model(x) - Y[j]).abs().mean().item() * len(j))
        trl, hol = sum(tl) / len(tr), sum(hl) / max(len(ho), 1)
        hist.append(dict(epoch=ep, train_l1=trl, heldout_l1=hol))
        print(f"epoch {ep:3d} train L1 {trl:.4f} held-out L1 {hol:.4f}", flush=True)
        if hol < best[0] - 1e-5:
            best = (hol, {k: v.clone() for k, v in model.state_dict().items()}, ep); bad = 0
        else:
            bad += 1
            if bad >= a.patience:
                break
    model.load_state_dict(best[1])
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    M.save(model, a.out, runs=a.runs, episodes=len(eps), best_epoch=best[2])
    rep = dict(runs=a.runs, episodes=len(eps), train_episodes=len(tr_ep), heldout_episodes=len(ho_ep),
               train_samples=int(len(tr)), heldout_samples=int(len(ho)), best_epoch=best[2], best_heldout_l1=best[0],
               zero_action_heldout_l1=float(Y[ho].abs().mean()), wall_s=round(time.time() - t0, 1), history=hist)
    json.dump(rep, open(Path(a.out).with_suffix(".json"), "w"), indent=1)
    print(json.dumps({k: v for k, v in rep.items() if k != "history"}))


if __name__ == "__main__":
    main()
