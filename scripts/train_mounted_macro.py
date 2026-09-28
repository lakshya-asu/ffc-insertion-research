"""Mounted-macro DINOv3 head; exact ROS preprocessing and validation-only selection."""

import argparse
import hashlib
import json
import random
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from dinov3_features import encode_dinov3 as encode
from dinov3_features import load_dinov3
from entrance_feature_model import CLASSES, FeatureHead
from ffc_cell.cv_frontend import REVISION, Frontend
from PIL import Image
from torch.nn import functional as F


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
    calibration = json.loads((a.data / "sensor/camera.json").read_text())
    assert calibration["preprocessing_revision"] == REVISION
    assert calibration["profile"] == "macro-mount-elevation45-v1"
    assert not any(calibration["d"]), "Mask rectification must be implemented for nonzero distortion"
    frontend = Frontend()
    images = np.stack(
        [
            frontend.process(
                np.array(Image.open(a.data / "sensor" / r["file"])),
                calibration["k"],
                calibration["d"],
                calibration["distortion_model"],
            )[0]
            for r in rows
        ]
    )
    masks = np.stack(
        [
            cv2.copyMakeBorder(
                cv2.resize(
                    np.array(Image.open(a.data / "offline" / r["file"])),
                    (1224, 1024),
                    interpolation=cv2.INTER_NEAREST_EXACT,
                ),
                0,
                0,
                4,
                4,
                cv2.BORDER_CONSTANT,
                value=0,
            )
            for r in rows
        ]
    )
    backbone = load_dinov3(a.backbone, device)
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
    for member, seed in enumerate([930301]):
        torch.manual_seed(seed)
        random.seed(seed)
        head = FeatureHead().to(device)
        opt = torch.optim.AdamW(head.parameters(), lr=0.001, weight_decay=0.0001)
        scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(opt, 60, eta_min=0.00003)
        frequency = np.bincount(masks[train].ravel(), minlength=6)
        class_weights = torch.tensor(
            np.sqrt(frequency.sum() / (6 * np.maximum(frequency, 1))), device=device, dtype=torch.float32
        ).clamp(max=50)
        dice_weights = torch.tensor([1, 1, 2, 1, 1], device=device)
        best = -1
        history = []
        for epoch in range(60):
            head.train()
            order = train.copy()
            random.shuffle(order)
            losses = []
            for pos in range(0, len(order), 2):
                x, y, z = batch(order[pos : pos + 2])
                opt.zero_grad(set_to_none=True)
                with torch.autocast("cuda", dtype=torch.bfloat16):
                    logits = head(x, z)
                prob = logits.float().softmax(1)
                truth = F.one_hot(y, 6).permute(0, 3, 1, 2)
                count = truth.sum((2, 3))
                pt = (prob * truth).sum(1).clamp_min(1e-7)
                ce = (-((1 - pt) ** 2) * pt.log() * class_weights[y]).mean()
                dice = (2 * (prob * truth).sum((2, 3)) + 1) / (prob.sum((2, 3)) + count + 1)
                loss = ce + ((1 - dice[:, 1:]) * dice_weights[None]).mean()
                loss.backward()
                opt.step()
                losses.append(float(loss.detach()))
            scheduler.step()
            head.eval()
            cm = np.zeros((6, 6), dtype=np.int64)
            mini_scores = []
            with torch.inference_mode():
                for pos in range(0, len(val), 2):
                    x, y, z = batch(val[pos : pos + 2])
                    with torch.autocast("cuda", dtype=torch.bfloat16):
                        pred = head(x, z).argmax(1)
                    cm += torch.bincount((y * 6 + pred).flatten(), minlength=36).cpu().numpy().reshape(6, 6)
                    for gt, pr in zip(y, pred, strict=True):
                        for cls in [1, 2, 3, 4, 5]:
                            if (gt == cls).sum() >= 8:
                                mini_scores.append(
                                    float(
                                        ((gt == cls) & (pr == cls)).sum() / ((gt == cls) | (pr == cls)).sum()
                                    )
                                )
            tp = cm.diagonal()
            union = cm.sum(0) + cm.sum(1) - tp
            iou = np.divide(tp, union, out=np.zeros(6), where=union > 0)
            score = float(np.mean(mini_scores))
            row = {
                "member": member,
                "epoch": epoch + 1,
                "loss": float(np.mean(losses)),
                "foreground_mean_iou": float(iou[1:].mean()),
                "validation_iou": iou.tolist(),
                "feature_mean_per_visible_image_iou": score,
                "feature_visible_iou_at_least_half": sum(v >= 0.5 for v in mini_scores),
                "feature_visible_images": len(mini_scores),
            }
            history.append(row)
            print(json.dumps(row), flush=True)
            if score > best:
                best = score
                torch.save(
                    {
                        "state_dict": {k: v.cpu() for k, v in head.state_dict().items()},
                        "classes": CLASSES,
                        "preprocessing_revision": REVISION,
                        "input_size": [1232, 1024],
                        "camera_profile": calibration["profile"],
                        "use_dino": True,
                        "epoch": epoch + 1,
                        "seed": seed,
                    },
                    a.output / f"member{member}.pt",
                )
            (a.output / f"member{member}-history.json").write_text(json.dumps(history, indent=2))
            (a.output / "history.json").write_text(json.dumps(history, indent=2))
        summary[f"member{member}"] = {
            "best_validation_feature_mean_per_visible_image_iou": best,
            "sha256": hashlib.sha256((a.output / f"member{member}.pt").read_bytes()).hexdigest(),
            "seed": seed,
        }
        del head
        torch.cuda.empty_cache()
    report = {
        "members": summary,
        "backbone": "official DINOv3 ViT-B/16",
        "scope": "Mounted-camera static simulation; engineered socket, no verified pose or action",
        "preprocessing_revision": REVISION,
        "camera_profile": calibration["profile"],
        "input_size": [1232, 1024],
        "batch_size": 2,
        "precision": "bfloat16 convolutions, float32 loss",
        "loss": "sqrt-frequency focal CE (gamma 2) plus Dice; validation-selected candidate",
        "epochs": 60,
        "train_images": len(train),
        "validation_images": len(val),
        "elapsed_s": time.monotonic() - start,
        "motion_permitted": False,
        "selection": "validation feature mean per-visible-image IoU",
        "labels_sha256": hashlib.sha256((a.data / "offline/labels.json").read_bytes()).hexdigest(),
        "backbone_sha256": hashlib.sha256((a.backbone / "model.safetensors").read_bytes()).hexdigest(),
        "calibration_or_test_used_for_training": False,
    }
    (a.output / "training-report.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
