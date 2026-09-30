"""Train the rung-1 imitation policy on an expert dataset from collect_expert.py.

Only ticks with target == 1 (the expert's own moves) are training targets; the perturbed ticks still appear in
the history window as context. Loss: mean squared error on the six actions, weighted 3x on the forward axis and
the lateral axis where the retract-and-shift decisions live.

usage: train_bc.py --data out/dataset_expert_zero_tape_6dof_good_L1 --out out/bc_zero_tape --epochs 30
"""
import argparse
import glob
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffc_twin.policy import ACT_DIM, HISTORY, OBS_DIM, build_mlp, stack_history  # noqa: E402


def load(data_dir: str, k: int, max_shards: int | None):
    X, Y, W = [], [], []
    for f in sorted(glob.glob(f"{data_dir}/shard_*.npz"))[:max_shards]:
        z = np.load(f)
        obs, act, target, offsets = z["obs"], z["act"], z["target"], z["offsets"]
        for i in range(len(offsets) - 1):
            s, e = offsets[i], offsets[i + 1]
            X.append(stack_history(obs[s:e], k)); Y.append(act[s:e]); W.append(target[s:e])
    return np.concatenate(X), np.concatenate(Y), np.concatenate(W)


def main():
    import torch

    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=30); ap.add_argument("--batch", type=int, default=4096); ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--hidden", type=int, default=256); ap.add_argument("--layers", type=int, default=3); ap.add_argument("--history", type=int, default=HISTORY)
    ap.add_argument("--max-shards", type=int, default=None); ap.add_argument("--val-frac", type=float, default=0.05)
    a = ap.parse_args()
    dev = "mps" if torch.backends.mps.is_available() else "cpu"
    t0 = time.time()
    X, Y, W = load(a.data, a.history, a.max_shards)
    print(f"loaded {len(X)} ticks, {int(W.sum())} expert-target ticks, in {time.time()-t0:.0f}s")
    mean = X[:, :OBS_DIM].mean(0); std = X[:, :OBS_DIM].std(0) + 1e-6
    Xn = (X - np.tile(mean, a.history)) / np.tile(std, a.history)
    n = len(Xn); idx = np.random.default_rng(0).permutation(n); nv = int(n * a.val_frac)
    vi, ti = idx[:nv], idx[nv:]
    Xt = torch.as_tensor(Xn, dtype=torch.float32, device=dev); Yt = torch.as_tensor(Y, dtype=torch.float32, device=dev); Wt = torch.as_tensor(W, dtype=torch.float32, device=dev)
    axis_w = torch.tensor([3.0, 3.0, 1.0, 1.0, 1.0, 1.0], device=dev)
    net = build_mlp(OBS_DIM * a.history, a.hidden, a.layers).to(dev)
    opt = torch.optim.AdamW(net.parameters(), lr=a.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.epochs)
    ti_t = torch.as_tensor(ti, device=dev); vi_t = torch.as_tensor(vi, device=dev)

    def loss_on(ix):
        pred = net(Xt[ix]); err = ((pred - Yt[ix]) ** 2 * axis_w).sum(1)
        return (err * Wt[ix]).sum() / Wt[ix].sum().clamp(min=1)

    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    best = float("inf"); log = []
    for ep in range(a.epochs):
        net.train(); perm = ti_t[torch.randperm(len(ti_t), device=dev)]; tl = 0.0; nb = 0
        for b in range(0, len(perm), a.batch):
            ix = perm[b : b + a.batch]; opt.zero_grad(); l = loss_on(ix); l.backward(); opt.step(); tl += float(l); nb += 1
        sched.step(); net.eval()
        with torch.no_grad():
            vl = float(sum(float(loss_on(vi_t[b : b + 16384])) * len(vi_t[b : b + 16384]) for b in range(0, len(vi_t), 16384)) / max(len(vi_t), 1))
        log.append(dict(epoch=ep, train=tl / nb, val=vl)); print(f"epoch {ep:3d} train {tl/nb:.4f} val {vl:.4f} {time.time()-t0:.0f}s")
        if vl < best:
            best = vl; torch.save(net.state_dict(), out / "weights.pt")
    (out / "meta.json").write_text(json.dumps(dict(obs_mean=mean.tolist(), obs_std=std.tolist(), history=a.history, hidden=a.hidden, layers=a.layers,
                                                   data=str(a.data), epochs=a.epochs, best_val=best, log=log, obs_dim=OBS_DIM, act_dim=ACT_DIM), indent=1))
    print("saved", out, "best val", round(best, 4))


if __name__ == "__main__":
    main()
