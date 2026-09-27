"""Publish pixel-identical Zero 2 W frames and a labelled static review montage."""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("run", type=Path)
p.add_argument("web", type=Path)
a = p.parse_args()
r = json.loads((a.run / "review.json").read_text())
assert r["timeline_elapsed_s"] == 0 and not r["motion_permitted"]
assert len(r["frames"]) == 12
assert {(f["case"], f["camera"]) for f in r["frames"]} == {
    (c, v["id"]) for c in ["loose", "approach", "near"] for v in r["configuration"]["cameras"]
}
a.web.mkdir(parents=True, exist_ok=True)
for frame in r["frames"]:
    src = Image.open(a.run / frame["file"]).convert("RGB")
    assert src.size == (3840, 2160)
    dest = a.web / (Path(frame["file"]).stem + ".webp")
    src.save(dest, lossless=True)
    assert np.array_equal(np.array(src), np.array(Image.open(dest)))
(a.run / "montage").mkdir(exist_ok=True)
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 17)
for i, (case, label) in enumerate(
    [
        ("loose", "Loose cable / connector inspection"),
        ("approach", "15 mm horizontal approach probe"),
        ("near", "3 mm horizontal approach probe"),
    ]
):
    canvas = Image.new("RGB", (1600, 1058), "#f2f3ed")
    d = ImageDraw.Draw(canvas)
    d.text((24, 18), "PI ZERO 2 W  /  " + label, font=font, fill="#162925")
    d.text(
        (24, 55),
        "Static authored poses. No grasp or insertion. Bottom-right: virtual macro for CAD inspection only.",
        font=small,
        fill="#45554d",
    )
    for j, view in enumerate(r["configuration"]["cameras"]):
        im = Image.open(a.run / f"{case}-{view['id']}.png")
        im.thumbnail((784, 441), Image.Resampling.LANCZOS)
        x, y = (j % 2) * 800, 96 + (j // 2) * 481
        canvas.paste(im, (x + 8, y))
        d.text((x + 15, y + 445), view["name"], font=small, fill="#162925")
    canvas.save(a.run / "montage" / f"{i:04d}.png")
r["asset_audit"] = {"lossless_images": 12, "all_match_source_pixels": True}
r["render_asset_license"] = "CC BY-SA 4.0; adapted Optocam Zero CAD, Doruk Kumkumoglu; see ATTRIBUTION.txt"
(a.web / "review.json").write_text(json.dumps(r, indent=2) + "\n")
