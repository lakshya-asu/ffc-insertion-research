"""Train two RGB-only-input heads with per-image emphasis on the small cable end."""

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
    a = p.parse_args()
    if a.output.exists() and any(a.output.iterdir()):
        raise ValueError("Fresh output required")
    a.output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(4)
    device = "cuda"
    rows = json.loads((a.data / "offline/labels.json").read_text())["frames"]
    assert set(r["split"] for r in rows) == {"train", "validation"}
    train = [i for i, r in enumerate(rows) if r["split"] == "train"]
    val = [i for i, r in enumerate(rows) if r["split"] == "validation"]
    assert not {rows[i]["scene"] for i in train} & {rows[i]["scene"] for i in val}
    images = np.stack(
        [np.array(Image.open(a.data / "sensor" / r["file"]).convert("RGB").crop(CROP)) for r in rows]
    )
    masks = np.stack([np.array(Image.open(a.data / "offline" / r["file"])) for r in rows])
    backbone = load_backbone(a.backbone / "source", a.backbone / "dinov2_vitb14_pretrain.pth", device)
    features = []
    start = time.monotonic()
    with torch.inference_mode():
        for i in range(len(rows)):
            x = torch.from_numpy(images[i].copy()).permute(2, 0, 1)[None].to(device).float() / 255
            with torch.autocast("cuda", dtype=torch.bfloat16):
                features.append(encode(backbone, x)[0].half().cpu())
            if i % 32 == 0:
                print("FEATURES", i, len(rows), flush=True)
    features = torch.stack(features)
    del backbone
    torch.cuda.empty_cache()

    def batch(ids):
        x = torch.from_numpy(images[ids].copy()).permute(0, 3, 1, 2).to(device).float() / 255
        y = torch.from_numpy(masks[ids].copy()).long().to(device)
        return x, y, features[ids].float().to(device)

    summary = {}
    for member, seed in enumerate([928611, 928612]):
        torch.manual_seed(seed)
        random.seed(seed)
        head = RegionHead(True).to(device)
        opt = torch.optim.AdamW(head.parameters(), lr=0.001, weight_decay=0.0001)
        scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(opt, 60, eta_min=0.00003)
        class_weights = torch.tensor([0.2, 1, 1, 2, 1], device=device)
        best = -1
        history = []
        for epoch in range(60):
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
                count = truth.sum((2, 3))
                nll = -(logits.log_softmax(1) * truth).sum((2, 3)) / count.clamp_min(1)
                weights = (count > 0) * class_weights[None]
                ce = ((nll * weights).sum(1) / weights.sum(1)).mean()
                dice = (2 * (prob * truth).sum((2, 3)) + 1) / (prob.sum((2, 3)) + count + 1)
                loss = ce + ((1 - dice[:, 1:]) * class_weights[None, 1:]).mean()
                loss.backward()
                opt.step()
                losses.append(float(loss.detach()))
            scheduler.step()
            head.eval()
            cm = np.zeros((5, 5), dtype=np.int64)
            mini_scores = []
            with torch.inference_mode():
                for pos in range(0, len(val), 4):
                    x, y, z = batch(val[pos : pos + 4])
                    pred = head(x, z).argmax(1)
                    cm += torch.bincount((y * 5 + pred).flatten(), minlength=25).cpu().numpy().reshape(5, 5)
                    for gt, pr in zip(y, pred, strict=True):
                        if (gt == 3).sum() >= 16:
                            mini_scores.append(
                                float(((gt == 3) & (pr == 3)).sum() / ((gt == 3) | (pr == 3)).sum())
                            )
            tp = cm.diagonal()
            union = cm.sum(0) + cm.sum(1) - tp
            iou = np.divide(tp, union, out=np.zeros(5), where=union > 0)
            score = float(np.mean(mini_scores))
            row = {
                "member": member,
                "epoch": epoch + 1,
                "loss": float(np.mean(losses)),
                "foreground_mean_iou": float(iou[1:].mean()),
                "validation_iou": iou.tolist(),
                "mini_end_mean_per_visible_image_iou": score,
                "mini_end_visible_iou_at_least_half": sum(v >= 0.5 for v in mini_scores),
                "mini_end_visible_images": len(mini_scores),
            }
            history.append(row)
            print(json.dumps(row), flush=True)
            if score > best:
                best = score
                torch.save(
                    {
                        "state_dict": {k: v.cpu() for k, v in head.state_dict().items()},
                        "classes": CLASSES,
                        "crop_xyxy": CROP,
                        "use_dino": True,
                        "epoch": epoch + 1,
                        "seed": seed,
                    },
                    a.output / f"member{member}.pt",
                )
            (a.output / f"member{member}-history.json").write_text(json.dumps(history, indent=2))
            (a.output / "history.json").write_text(json.dumps(history, indent=2))
        summary[f"member{member}"] = {
            "best_validation_mini_end_mean_per_visible_image_iou": best,
            "sha256": hashlib.sha256((a.output / f"member{member}.pt").read_bytes()).hexdigest(),
            "seed": seed,
        }
        del head
        torch.cuda.empty_cache()
    report = {
        "members": summary,
        "epochs": 60,
        "train_images": len(train),
        "validation_images": len(val),
        "elapsed_s": time.monotonic() - start,
        "motion_permitted": False,
        "selection": "validation mini-end mean per-visible-image IoU",
        "labels_sha256": hashlib.sha256((a.data / "offline/labels.json").read_bytes()).hexdigest(),
        "backbone_sha256": hashlib.sha256(
            (a.backbone / "dinov2_vitb14_pretrain.pth").read_bytes()
        ).hexdigest(),
        "calibration_or_test_used_for_training": False,
    }
    (a.output / "training-report.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
