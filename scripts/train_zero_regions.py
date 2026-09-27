"""Fit a DINOv2 visible-region head and an RGB-only ablation on development data only."""

import argparse
import hashlib
import json
import random
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from torch.nn import functional as F
from zero_region_model import CLASSES, CROP, RegionHead, encode, load_backbone


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--data", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--backbone", type=Path, default=Path("/backbone"))
    p.add_argument("--epochs", type=int, default=40)
    a = p.parse_args()
    if a.output.exists() and any(a.output.iterdir()):
        raise ValueError("Use fresh output")
    a.output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(4)
    torch.manual_seed(927611)
    random.seed(927611)
    np.random.seed(927611)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    if device != "cuda":
        raise RuntimeError("GPU required for this recorded experiment")
    rows = json.loads((a.data / "offline/labels.json").read_text())["frames"]
    if set(r["split"] for r in rows) != {"train", "validation"}:
        raise ValueError("Development data only")
    train = [i for i, r in enumerate(rows) if r["split"] == "train"]
    val = [i for i, r in enumerate(rows) if r["split"] == "validation"]
    if {rows[i]["scene"] for i in train} & {rows[i]["scene"] for i in val}:
        raise ValueError("Scene leakage")
    images = np.stack(
        [np.array(Image.open(a.data / "sensor" / r["file"]).convert("RGB").crop(CROP)) for r in rows]
    )
    masks = np.stack([np.array(Image.open(a.data / "offline" / r["file"])) for r in rows])
    assert images.shape[1:] == (448, 896, 3) and masks.shape[1:] == (448, 896)
    for subset in [train, val]:
        assert min(np.bincount(masks[subset].ravel(), minlength=5)[1:]) > 0
    backbone = load_backbone(a.backbone / "source", a.backbone / "dinov2_vitb14_pretrain.pth", device)
    cache = []
    started = time.monotonic()
    with torch.inference_mode():
        for i in range(len(rows)):
            rgb = torch.from_numpy(images[i].copy()).permute(2, 0, 1)[None].to(device).float() / 255
            with torch.autocast("cuda", dtype=torch.bfloat16):
                feature = encode(backbone, rgb)
            cache.append(feature[0].half().cpu())
            if i % 16 == 0:
                print("FEATURES", i, len(rows), flush=True)
    features = torch.stack(cache)
    del cache, backbone
    torch.cuda.empty_cache()
    summary = {}
    for use_dino, name in [(True, "dino"), (False, "rgb_ablation")]:
        torch.manual_seed(927611)
        random.seed(927611)
        head = RegionHead(use_dino).to(device)
        opt = torch.optim.AdamW(head.parameters(), lr=0.001, weight_decay=0.0001)
        scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.epochs, eta_min=0.00005)
        weights = torch.tensor([0.15, 4, 1, 6, 2], device=device)
        best = -1
        history = []

        def batch(indices, use_dino=use_dino):
            x = torch.from_numpy(images[indices].copy()).permute(0, 3, 1, 2).to(device).float() / 255
            y = torch.from_numpy(masks[indices].copy()).long().to(device)
            z = features[indices].float().to(device) if use_dino else None
            return x, y, z

        for epoch in range(a.epochs):
            head.train()
            order = train.copy()
            random.shuffle(order)
            losses = []
            for pos in range(0, len(order), 4):
                x, y, z = batch(order[pos : pos + 4])
                opt.zero_grad(set_to_none=True)
                logits = head(x, z)
                prob = logits.softmax(1)
                truth = F.one_hot(y, 5).permute(0, 3, 1, 2)
                dice = (2 * (prob * truth).sum((0, 2, 3)) + 1) / (
                    prob.sum((0, 2, 3)) + truth.sum((0, 2, 3)) + 1
                )
                loss = F.cross_entropy(logits, y, weight=weights) + (1 - dice[1:]).mean()
                loss.backward()
                opt.step()
                losses.append(float(loss.detach()))
            scheduler.step()
            head.eval()
            cm = np.zeros((5, 5), dtype=np.int64)
            with torch.inference_mode():
                for pos in range(0, len(val), 4):
                    x, y, z = batch(val[pos : pos + 4])
                    pred = head(x, z).argmax(1)
                    cm += torch.bincount((y * 5 + pred).flatten(), minlength=25).cpu().numpy().reshape(5, 5)
            tp = cm.diagonal()
            union = cm.sum(0) + cm.sum(1) - tp
            iou = np.divide(tp, union, out=np.zeros(5), where=union > 0)
            score = float(iou[1:].mean())
            row = {
                "epoch": epoch + 1,
                "loss": float(np.mean(losses)),
                "foreground_mean_iou": score,
                "validation_iou": iou.tolist(),
            }
            history.append(row)
            print(name, json.dumps(row), flush=True)
            if score > best:
                best = score
                torch.save(
                    {
                        "state_dict": {k: v.cpu() for k, v in head.state_dict().items()},
                        "classes": CLASSES,
                        "use_dino": use_dino,
                        "crop_xyxy": CROP,
                        "epoch": epoch + 1,
                    },
                    a.output / (name + ".pt"),
                )
            (a.output / (name + "-history.json")).write_text(json.dumps(history, indent=2))
            if use_dino:
                (a.output / "history.json").write_text(json.dumps(history, indent=2))
        summary[name] = {
            "best_validation_foreground_mean_iou": best,
            "model_sha256": hashlib.sha256((a.output / (name + ".pt")).read_bytes()).hexdigest(),
        }
        del head
        torch.cuda.empty_cache()
    report = {
        "models": summary,
        "epochs": a.epochs,
        "train_images": len(train),
        "validation_images": len(val),
        "elapsed_s": time.monotonic() - started,
        "device": device,
        "classes": CLASSES,
        "motion_permitted": False,
        "backbone": "frozen DINOv2 ViT-B/14, blocks 6 and 12; native RGB detail decoder",
        "backbone_sha256": hashlib.sha256(
            (a.backbone / "dinov2_vitb14_pretrain.pth").read_bytes()
        ).hexdigest(),
        "labels_sha256": hashlib.sha256((a.data / "offline/labels.json").read_bytes()).hexdigest(),
        "test_used_for_training_or_selection": False,
        "seed": 927611,
    }
    (a.output / "training-report.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
