"""Isolated RGB inference for frozen two-head review, with an optional frozen gate."""

import hashlib
import json
import time
from pathlib import Path

import numpy as np
import torch
from camera_observations import CameraGate, CameraSpec
from PIL import Image, ImageFilter
from zero_region_model import CLASSES, CROP, RegionHead, encode, load_backbone
from zero_reliability import apply_review_gate, review_evidence


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    sensor, models, old, output, backbone_dir = map(
        Path, ["/sensor", "/models", "/previous", "/results", "/backbone"]
    )
    if any(output.iterdir()):
        raise ValueError("Fresh inference output required")
    torch.set_num_threads(4)
    device = "cuda"
    specs = json.loads((sensor / "calibration.json").read_text())
    for spec in specs.values():
        payload = {k: v for k, v in spec.items() if k != "calibration_id"}
        assert (
            hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest() == spec["calibration_id"]
        )
    runtime_hashes = {
        f: sha(Path("/code") / f)
        for f in [
            "zero_reliable_worker.py",
            "zero_reliability.py",
            "zero_region_model.py",
            "camera_observations.py",
        ]
    }
    backbone_hash = sha(backbone_dir / "dinov2_vitb14_pretrain.pth")
    gate = CameraGate({k: CameraSpec(v["width"], v["height"], v["calibration_id"]) for k, v in specs.items()})
    backbone = load_backbone(backbone_dir / "source", backbone_dir / "dinov2_vitb14_pretrain.pth", device)
    heads, hashes = {}, {}
    for name, path in [
        ("member0", models / "member0.pt"),
        ("member1", models / "member1.pt"),
        ("previous", old / "dino.pt"),
    ]:
        checkpoint = torch.load(path, map_location="cpu", weights_only=True)
        assert tuple(checkpoint["classes"]) == CLASSES and tuple(checkpoint["crop_xyxy"]) == CROP
        head = RegionHead(True)
        head.load_state_dict(checkpoint["state_dict"])
        heads[name] = head.to(device).eval()
        hashes[name] = sha(path)
    policy_file = Path("/policy/policy.json")
    policy = json.loads(policy_file.read_text()) if policy_file.is_file() else {"thresholds": {}}
    if policy_file.is_file():
        assert policy["model_sha256"] == hashes
        assert policy["backbone_sha256"] == backbone_hash
        assert policy["runtime_code_sha256"] == runtime_hashes
        assert policy["calibration_ids"] == {k: v["calibration_id"] for k, v in specs.items()}
        assert policy["evidence_code_sha256"] == sha(Path("/code/zero_reliability.py"))
    for name in ["ensemble", "previous"]:
        (output / name).mkdir()
    rows = json.loads((sensor / "frames.json").read_text())["frames"]
    results, stress = [], []
    stress_enabled = policy_file.is_file()
    stress_candidates = {}

    def predict(rgb):
        x = torch.from_numpy(rgb.copy()).permute(2, 0, 1)[None].to(device).float() / 255
        torch.cuda.synchronize()
        started = time.perf_counter()
        with torch.inference_mode():
            with torch.autocast("cuda", dtype=torch.bfloat16):
                z = encode(backbone, x)
            z = z.float()
            members = [heads[n](x, z).softmax(1)[0] for n in ["member0", "member1"]]
            probability = (members[0] + members[1]) / 2
            member_masks = torch.stack([v.argmax(0) for v in members]).cpu().numpy().astype(np.uint8)
            probs = probability.cpu().numpy()
            torch.cuda.synchronize()
            elapsed = time.perf_counter() - started
            previous = heads["previous"](x, z).argmax(1)[0].cpu().numpy().astype(np.uint8)
        evidence = review_evidence(probs, member_masks, rgb)
        return probs.argmax(0).astype(np.uint8), previous, evidence, elapsed

    for i, item in enumerate(rows):
        path = sensor / item["file"]
        assert path.parent == sensor and path.suffix == ".png" and sha(path) == item["sha256"]
        raw = np.array(Image.open(path).convert("RGB"))
        packet = {
            k: item[k] for k in ["camera_id", "sequence", "timestamp_s", "width", "height", "calibration_id"]
        }
        packet["rgb"] = raw.tobytes()
        gate.accept(packet, now_s=item["timestamp_s"])
        assert tuple(specs[item["camera_id"]]["crop_xyxy"]) == CROP
        x0, y0, x1, y1 = CROP
        rgb = raw[y0:y1, x0:x1]
        mask, previous, evidence, elapsed = predict(rgb)
        Image.fromarray(mask).save(output / "ensemble" / item["file"])
        Image.fromarray(previous).save(output / "previous" / item["file"])
        decision = apply_review_gate(evidence, item["camera_id"], policy)
        results.append(
            {
                "file": item["file"],
                "camera_id": item["camera_id"],
                "evidence": evidence,
                "decision": decision,
                "compute_s": elapsed,
            }
        )
        # Select a strong RGB-derived view, so faults are not tested only on an
        # already empty or poor frame. No labels are available here.
        if stress_enabled:
            rank = (evidence["eligible"], evidence["score"])
            old_candidate = stress_candidates.get(item["camera_id"])
            if old_candidate is None or rank > old_candidate[0]:
                stress_candidates[item["camera_id"]] = (rank, rgb.copy(), item["file"], decision)
        if i % 16 == 0:
            print("REPLAY", i, len(rows), flush=True)
    for camera_id, (_, rgb, filename, original_decision) in stress_candidates.items():
        for fault, corrupted in [
            ("blackout", np.zeros_like(rgb)),
            ("whiteout", np.full_like(rgb, 255)),
            ("severe_blur", np.array(Image.fromarray(rgb).filter(ImageFilter.GaussianBlur(12)))),
        ]:
            _, _, ev, _ = predict(corrupted)
            stress.append(
                {
                    "file": filename,
                    "camera_id": camera_id,
                    "fault": fault,
                    "original_decision": original_decision,
                    "evidence": ev,
                    "decision": apply_review_gate(ev, camera_id, policy),
                }
            )
    report = {
        "frames": results,
        "classes": CLASSES,
        "crop_xyxy": CROP,
        "model_sha256": hashes,
        "backbone_sha256": backbone_hash,
        "runtime_code_sha256": runtime_hashes,
        "peak_cuda_allocated_mb_including_comparison": torch.cuda.max_memory_allocated() / 1024**2,
        "evidence_code_sha256": sha(Path("/code/zero_reliability.py")),
        "calibration_ids": {k: v["calibration_id"] for k, v in specs.items()},
        "policy_sha256": sha(policy_file) if policy_file.is_file() else None,
        "motion_permitted": False,
        "timing_scope": (
            "shared encoder + two heads; excludes decode, component scoring and previous-model comparison"
        ),
        "clock_mode": "recorded-clock replay, not live freshness demonstration",
        "stress": stress,
    }
    (output / "predictions.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
