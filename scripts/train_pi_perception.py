"""Train/validation only. Hold-out test labels are not mounted in this process."""

import argparse
import hashlib
import json
import random
import time
from pathlib import Path

import numpy as np
import torch
from pi_perception_model import CLASSES, PiPerception
from PIL import Image
from torch.nn import functional as F


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--data", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--epochs", type=int, default=40)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    seed = 927311
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.set_num_threads(4)
    torch.backends.cudnn.benchmark = False
    torch.use_deterministic_algorithms(True, warn_only=True)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    frames = json.loads((a.data / "offline/labels.json").read_text())["frames"]
    if {r["split"] for r in frames} != {"train", "validation"}:
        raise ValueError("Only development data allowed")
    images = np.stack([np.asarray(Image.open(a.data / "sensor" / r["file"])) for r in frames])
    masks = np.stack([np.asarray(Image.open(a.data / "offline" / r["file"])) for r in frames])
    for split in ["train", "validation"]:
        selected = masks[[i for i, r in enumerate(frames) if r["split"] == split]]
        counts = np.bincount(selected.ravel(), minlength=6)
        if min(counts[1:]) == 0:
            raise ValueError(f"{split} has empty task classes: {counts.tolist()}")
    train = [i for i, r in enumerate(frames) if r["split"] == "train"]
    val = [i for i, r in enumerate(frames) if r["split"] == "validation"]
    if {frames[i]["file"].split("-")[0] for i in train} & {frames[i]["file"].split("-")[0] for i in val}:
        raise ValueError("Scene leakage between train and validation")
    model = PiPerception().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=0.001, weight_decay=0.0001)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, a.epochs, eta_min=0.00008)
    weights = torch.tensor([0.2, 1, 8, 3, 5, 5], device=device)
    best, history, start = -1.0, [], time.monotonic()

    def batch(indices, augment):
        x = torch.from_numpy(images[indices].copy()).permute(0, 3, 1, 2).float().to(device) / 255
        y = torch.from_numpy(masks[indices].copy()).long().to(device)
        if augment:
            for axis in [-1, -2]:
                if random.random() < 0.5:
                    x, y = x.flip(axis), y.flip(axis)
            gain = torch.rand((len(indices), 1, 1, 1), device=device) * 0.3 + 0.85
            x = (x * gain + torch.randn_like(x) * 0.008).clamp(0, 1)
        return x, y

    for epoch in range(a.epochs):
        model.train()
        random.shuffle(train)
        losses = []
        for offset in range(0, len(train), 4):
            x, y = batch(train[offset : offset + 4], True)
            optimizer.zero_grad(set_to_none=True)
            logits = model(x)
            prob = logits.softmax(1)
            truth = F.one_hot(y, 6).permute(0, 3, 1, 2)
            dice = (2 * (prob * truth).sum((0, 2, 3)) + 1) / (prob.sum((0, 2, 3)) + truth.sum((0, 2, 3)) + 1)
            loss = F.cross_entropy(logits, y, weight=weights) + (1 - dice[1:]).mean()
            loss.backward()
            optimizer.step()
            losses.append(float(loss.detach()))
        scheduler.step()
        model.eval()
        confusion = np.zeros((6, 6), dtype=np.int64)
        with torch.no_grad():
            for offset in range(0, len(val), 4):
                x, y = batch(val[offset : offset + 4], False)
                pred = model(x).argmax(1)
                counts = torch.bincount((y * 6 + pred).flatten(), minlength=36)
                confusion += counts.cpu().numpy().reshape(6, 6)
        tp = confusion.diagonal()
        union = confusion.sum(0) + confusion.sum(1) - tp
        iou = np.divide(tp, union, out=np.zeros(6), where=union > 0)
        score = float(iou[1:].mean())
        row = {
            "epoch": epoch + 1,
            "loss": float(np.mean(losses)),
            "validation_iou": iou.tolist(),
            "foreground_mean_iou": score,
        }
        history.append(row)
        print(json.dumps(row), flush=True)
        if score > best:
            best = score
            torch.save(
                {
                    "state_dict": model.cpu().state_dict(),
                    "classes": CLASSES,
                    "epoch": epoch + 1,
                    "input": "RGB8 scaled to [0,1]; no calibration or simulator state",
                    "seed": seed,
                },
                a.output / "model.pt",
            )
            model.to(device)
        (a.output / "history.json").write_text(json.dumps(history, indent=2))
    report = {
        "seed": seed,
        "device": device,
        "torch": torch.__version__,
        "epochs": a.epochs,
        "train_images": len(train),
        "validation_images": len(val),
        "selection": "maximum validation foreground mean IoU",
        "best_validation_foreground_mean_iou": best,
        "elapsed_s": time.monotonic() - start,
        "model_sha256": hashlib.sha256((a.output / "model.pt").read_bytes()).hexdigest(),
        "labels_sha256": hashlib.sha256((a.data / "offline/labels.json").read_bytes()).hexdigest(),
        "test_used_for_training_or_selection": False,
        "motion_permitted": False,
    }
    (a.output / "training-report.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
