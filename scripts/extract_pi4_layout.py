"""Recover small rectangular package outlines from the official Pi4 vector drawing.

This adds footprint-aligned visual proxies, not a BOM or measured package heights.
"""

import collections
import hashlib
import json
from pathlib import Path

import pymupdf

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / "third_party/raspberry_pi/pi4-mechanical.pdf"
page = pymupdf.open(source)[0]
graph = collections.defaultdict(set)
for drawing in page.get_drawings():
    for kind, *points in drawing["items"]:
        if kind != "l":
            continue
        a, b = [tuple(round(v, 2) for v in point) for point in points]
        if a == b or (a[0] != b[0] and a[1] != b[1]):
            continue
        if all(149.52 < x < 643.33 and 260.87 < y < 586.21 for x, y in [a, b]):
            graph[a].add(b)
            graph[b].add(a)
seen, candidates = set(), []
for vertex in graph:
    if vertex in seen:
        continue
    todo, component = [vertex], []
    while todo:
        vertex = todo.pop()
        if vertex in seen:
            continue
        seen.add(vertex)
        component.append(vertex)
        todo.extend(graph[vertex] - seen)
    xs = sorted({p[0] for p in component})
    ys = sorted({p[1] for p in component})
    if len(xs) != 2 or len(ys) != 2 or len(component) != 4 or any(len(graph[p]) != 2 for p in component):
        continue
    x = (sum(xs) / 2 - 149.52) * 85 / (643.32 - 149.52)
    y = (586.2 - sum(ys) / 2) * 56 / (586.2 - 260.88)
    dx = (xs[1] - xs[0]) * 85 / (643.32 - 149.52)
    dy = (ys[1] - ys[0]) * 56 / (586.2 - 260.88)
    if 0.2 <= min(dx, dy) and max(dx, dy) <= 3.5:
        candidates.append([x, y, dx, dy])
major_parts = json.loads((ROOT / "third_party/raspberry_pi/pi4.json").read_text())["components"][1:]
kept = []
for x, y, dx, dy in candidates:
    blocked = False
    for part in major_parts:
        a, b = part["bounds_mm"]
        ax, ay = a[0] + 42.5, a[1] + 28.3520692157758
        bx, by = b[0] + 42.5, b[1] + 28.3520692157758
        if x + dx / 2 > ax and x - dx / 2 < bx and y + dy / 2 > ay and y - dy / 2 < by:
            blocked = True
    if not blocked:
        kept.append({"xy_mm": [round(x, 5), round(y, 5)], "size_xy_mm": [round(dx, 5), round(dy, 5)]})
result = {
    "source": "RP-008343-DS-1",
    "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
    "method": "Closed axis-aligned small rectangles; reject overlap with existing major CAD components",
    "count": len(kept),
    "units": "mm from PCB lower-left",
    "limitations": "Drawing-derived footprint proxies. Heights, package types and materials are assumed.",
    "packages": kept,
}
(ROOT / "config/pi4-detail-layout.json").write_text(json.dumps(result, indent=2) + "\n")
print("Extracted", len(kept), "footprint proxies")
