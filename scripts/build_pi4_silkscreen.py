"""Author vector text for recognizable board labels; placement is approximate."""

import json
from pathlib import Path

import mapbox_earcut
import numpy as np
from matplotlib.font_manager import FontProperties
from matplotlib.path import Path as Polygon
from matplotlib.textpath import TextPath

ROOT = Path(__file__).resolve().parents[1]
labels = [
    ("CAMERA", 43.0, 8.0, 1.1, 90),
    ("DISPLAY", 7.0, 24.0, 1.1, 90),
    ("HDMI0", 22.0, 5.7, 1.0, 0),
    ("HDMI1", 35.0, 5.7, 1.0, 0),
    ("PWR IN", 7.0, 5.7, 1.0, 0),
    ("GPIO", 26.0, 47.0, 1.0, 0),
    ("Raspberry Pi 4 Model B", 22.0, 42.5, 1.35, 0),
]
records = []
for text, x, y, size, angle in labels:
    loops = TextPath((0, 0), text, size=size, prop=FontProperties(family="DejaVu Sans")).to_polygons()
    loops = [a[:-1] if np.allclose(a[0], a[-1]) else a for a in loops]
    areas = [abs(np.sum(a[:, 0] * np.roll(a[:, 1], -1) - a[:, 1] * np.roll(a[:, 0], -1))) for a in loops]
    parents = []
    for i, a in enumerate(loops):
        choices = [j for j, b in enumerate(loops) if areas[j] > areas[i] and Polygon(b).contains_point(a[0])]
        parents.append(min(choices, key=lambda j: areas[j]) if choices else None)
    vertices, indices = [], []
    for i, a in enumerate(loops):
        if parents[i] is not None:
            continue
        rings = [a] + [loops[j] for j, parent in enumerate(parents) if parent == i]
        pts = np.concatenate(rings)
        tris = mapbox_earcut.triangulate_float64(pts, np.cumsum([len(r) for r in rings], dtype=np.uint32))
        indices.extend((tris + len(vertices)).tolist())
        rad = np.radians(angle)
        rotation = np.array([[np.cos(rad), -np.sin(rad)], [np.sin(rad), np.cos(rad)]])
        vertices.extend((pts @ rotation.T + [x, y]).round(6).tolist())
    records.append({"text": text, "xy_mm": vertices, "triangles": indices})
result = {
    "basis": "Human-readable labels; font and positions are approximations, not manufacturer artwork",
    "font": "DejaVu Sans",
    "labels": records,
}
(ROOT / "config/pi4-silkscreen.json").write_text(json.dumps(result, separators=(",", ":")) + "\n")
print("Vector labels:", len(records))
