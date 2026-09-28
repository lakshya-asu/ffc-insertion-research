"""Official pretrained SAM 3.1 text-prompt smoke check on RGB only; not a benchmark."""

import hashlib
import json
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from sam3.model_builder import build_sam3_predictor

out = Path("/results")
if (out / "report.json").exists():
    raise RuntimeError("Use a fresh results directory")
checkpoint = Path("/weights/sam3.1_multiplex.pt")
start = time.perf_counter()
predictor = build_sam3_predictor(
    checkpoint_path=str(checkpoint),
    version="sam3.1",
    compile=False,
    warm_up=False,
    use_fa3=False,
    async_loading_frames=False,
)
# Pinned upstream base predictor passes an unsupported false offload flag.
# Preserve default behavior; reject requests for unsupported state offloading.
original_init_state = predictor.model.init_state

def compatible_init_state(*args, offload_state_to_cpu=False, **kwargs):
    if offload_state_to_cpu:
        raise NotImplementedError("SAM 3.1 multiplex state offload is unsupported")
    return original_init_state(*args, **kwargs)

predictor.model.init_state = compatible_init_state
torch.cuda.synchronize()
load_seconds = time.perf_counter() - start
rows = []
for camera in ["entrance", "offset"]:
    source = Path("/sensor") / f"0000-{camera}.png"
    im = Image.open(source).convert("RGB").crop((1472, 856, 2368, 1304))
    frames = out / camera
    frames.mkdir()
    # The official video loader takes numbered JPEG frames. Each image starts
    # a separate session: independent static scenes are never treated as video.
    im.save(frames / "00000.jpg", quality=100, subsampling=0)
    for idx, prompt in enumerate(["ribbon cable", "ribbon cable connector", "blue cable stiffener"]):
        session = predictor.handle_request(
            dict(type="start_session", resource_path=str(frames), offload_video_to_cpu=True)
        )["session_id"]
        torch.cuda.synchronize()
        start = time.perf_counter()
        with torch.inference_mode():
            response = predictor.handle_request(
                dict(type="add_prompt", session_id=session, frame_index=0, text=prompt)
            )
        torch.cuda.synchronize()
        elapsed = time.perf_counter() - start
        data = response["outputs"]
        arrays = {
            k: (v.detach().cpu().numpy() if torch.is_tensor(v) else np.asarray(v))
            for k, v in data.items()
            if k in ["out_obj_ids", "out_probs", "out_binary_masks", "out_boxes_xywh"]
        }
        np.savez_compressed(out / f"{camera}-{idx}.npz", **arrays)
        masks = arrays["out_binary_masks"].astype(bool)
        rgb = np.array(Image.open(frames / "00000.jpg").convert("RGB"))
        palette = np.array([[240, 140, 35], [45, 180, 230], [60, 220, 130]], dtype=np.uint8)
        for j, mask in enumerate(masks):
            rgb[mask] = (rgb[mask] * 0.5 + palette[j % 3] * 0.5).astype(np.uint8)
        Image.fromarray(rgb).save(out / f"{camera}-{idx}.webp", lossless=True)
        rows.append(
            dict(
                camera=camera,
                prompt=prompt,
                objects=len(masks),
                mask_pixels=[int(m.sum()) for m in masks],
                uncalibrated_scores=arrays["out_probs"].tolist(),
                seconds=elapsed,
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            )
        )
        predictor.handle_request(dict(type="close_session", session_id=session))
        print(json.dumps(rows[-1]), flush=True)
report = dict(
    rows=rows,
    model="SAM 3.1 official pretrained multiplex",
    load_seconds=load_seconds,
    checkpoint_sha256=hashlib.file_digest(checkpoint.open("rb"), "sha256").hexdigest(),
    peak_cuda_allocated_bytes=torch.cuda.max_memory_allocated(),
    torch_version=torch.__version__,
    cuda=torch.version.cuda,
    crop_xyxy=[1472, 856, 2368, 1304],
    use_fa3=False,
    compiled=False,
    scope="Six fixed-text development probes, not accuracy evaluation or tracking validation",
    motion_permitted=False,
    labels_available_to_worker=False,
)
(out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
