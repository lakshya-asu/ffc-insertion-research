"""6-class contact classifier on the observation window (spec B5). Labels from scripts/label_contact.py (offline).

usage: python scripts/train_classifier.py out/episodes/<run> [more runs] [--out out/models/clf_v1.pt]

Cross entropy with class weights (inverse square root of class frequency), 20 percent of episodes held out, early
stop on held-out loss. Reports on held-out episodes: the per-tick confusion matrix, per-tick seat FP and FN, and the
seat rule of report 4.2 (P(seated) > 0.9 for 3 consecutive ticks): episode-level FP = it fires at a tick where truth
is not seated, FN = truth is seated at the end and it never fired on a seated tick. Wilson 95 percent intervals.
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
from cell_twin.training_config import CONTACT_CLASSES  # noqa: E402

SEATED = CONTACT_CLASSES.index("seated")


def wilson(k, n, z=1.96):
    if n == 0:
        return [None, None, None]
    p = k / n; d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d; h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [p, max(0.0, c - h), min(1.0, c + h)]


def windows(ds, j):
    return torch.from_numpy(ds.rows)[torch.from_numpy(ds.win_idx[j])].reshape(len(j), -1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out", default=str(ROOT / "out" / "models" / "clf_v1.pt"))
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--patience", type=int, default=5)
    ap.add_argument("--bs", type=int, default=1024)
    ap.add_argument("--threads", type=int, default=4)
    a = ap.parse_args()
    torch.set_num_threads(a.threads); torch.manual_seed(0)
    t0 = time.time()
    eps = D.list_episodes(a.runs)
    ds = D.load(eps, "clf")
    tr_ep, ho_ep = D.split_episodes(len(eps))
    tr = np.flatnonzero(np.isin(ds.episode, tr_ep)); ho = np.flatnonzero(np.isin(ds.episode, ho_ep))
    cnt = np.bincount(ds.label[tr], minlength=6).astype(float)
    w = np.where(cnt > 0, (cnt.sum() / (6 * np.maximum(cnt, 1))) ** 0.5, 0.0)
    model = M.ContactCNN(); M.set_norm(model, *D.row_stats(ds.rows))
    opt = torch.optim.Adam(model.parameters(), lr=1e-3)
    lossf = torch.nn.CrossEntropyLoss(weight=torch.tensor(w, dtype=torch.float32))
    Y = torch.from_numpy(ds.label)
    rng = np.random.default_rng(0); best = (np.inf, None, -1); bad = 0; hist = []
    for ep in range(a.epochs):
        model.train(); tl = 0.0
        order = rng.permutation(tr)
        for i in range(0, len(order), a.bs):
            j = order[i:i + a.bs]
            loss = lossf(model(windows(ds, j)), Y[j])
            opt.zero_grad(); loss.backward(); opt.step(); tl += loss.item() * len(j)
        model.eval(); hl = 0.0
        with torch.no_grad():
            for i in range(0, len(ho), 8192):
                j = ho[i:i + 8192]; hl += lossf(model(windows(ds, j)), Y[j]).item() * len(j)
        hist.append(dict(epoch=ep, train_ce=tl / len(tr), heldout_ce=hl / max(len(ho), 1)))
        print(f"epoch {ep:3d} train CE {tl / len(tr):.4f} held-out CE {hl / max(len(ho), 1):.4f}", flush=True)
        if hl < best[0] - 1e-6:
            best = (hl, {k: v.clone() for k, v in model.state_dict().items()}, ep); bad = 0
        else:
            bad += 1
            if bad >= a.patience:
                break
    model.load_state_dict(best[1]); model.eval()
    with torch.no_grad():
        P = torch.cat([torch.softmax(model(windows(ds, ho[i:i + 8192])), 1) for i in range(0, len(ho), 8192)]).numpy()
    y = ds.label[ho]; pred = P.argmax(1)
    conf = np.zeros((6, 6), int)
    np.add.at(conf, (y, pred), 1)
    not_seat = y != SEATED
    tick_fp = wilson(int(((pred == SEATED) & not_seat).sum()), int(not_seat.sum()))
    tick_fn = wilson(int(((pred != SEATED) & ~not_seat).sum()), int((~not_seat).sum()))
    # episode-level seat rule: P(seated) > 0.9 for 3 consecutive ticks
    fp = fn = n_nonseat = n_seat = 0; term_fp = term_fn = 0
    eph = ds.episode[ho]
    for e in np.unique(eph):
        s = eph == e
        ps = P[s, SEATED]; ye = y[s]
        fire = np.convolve((ps > 0.9).astype(int), np.ones(3, int), "full")[:len(ps)] >= 3
        truth_seated_end = ye[-1] == SEATED
        if truth_seated_end:
            n_seat += 1; fn += int(not (fire & (ye == SEATED)).any()); term_fn += int(pred[s][-1] != SEATED)
        else:
            n_nonseat += 1; term_fp += int(pred[s][-1] == SEATED)
        fp += int((fire & (ye != SEATED)).any())
    n_ep = len(np.unique(eph))
    rep = dict(runs=a.runs, episodes=len(eps), heldout_episodes=int(n_ep), heldout_ticks=int(len(ho)),
               class_counts_train=cnt.astype(int).tolist(), class_weights=w.tolist(), best_epoch=best[2],
               confusion_heldout=dict(classes=CONTACT_CLASSES, rows_truth_cols_pred=conf.tolist()),
               accuracy_heldout=float((pred == y).mean()),
               seat_tick_fp=tick_fp, seat_tick_fn=tick_fn,
               seat_rule_episode_fp=wilson(fp, n_ep), seat_rule_episode_fn=wilson(fn, n_seat),
               seat_terminal_fp=wilson(term_fp, n_nonseat), seat_terminal_fn=wilson(term_fn, n_seat),
               heldout_seated_episodes=n_seat, heldout_nonseated_episodes=n_nonseat,
               wall_s=round(time.time() - t0, 1), history=hist)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    M.save(model, a.out, runs=a.runs, episodes=len(eps), best_epoch=best[2])
    json.dump(rep, open(Path(a.out).with_suffix(".json"), "w"), indent=1)
    print(json.dumps({k: v for k, v in rep.items() if k != "history"}))


if __name__ == "__main__":
    main()
