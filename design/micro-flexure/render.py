"""CPU rendering of actual concept CAD; no generated imagery or physics claims."""

import argparse
import io
import json
import subprocess
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from build import box, mesh_data, parts
from matplotlib.animation import PillowWriter
from PIL import Image
from pxr import Usd, UsdGeom, UsdShade

ROOT = Path(__file__).resolve().parents[2]
BG = "#f3f2ee"
RASTER = ROOT / "outputs/micro-flexure-raster"


def solid(ax, vertices, faces, color, alpha=1):
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    light = np.array([-0.45, -0.4, 0.8])
    light /= np.linalg.norm(light)
    brightness = 0.48 + 0.52 * np.abs(normals @ light)
    colours = np.clip(brightness[:, None] * np.array(color), 0, 1)
    ax.review_triangles.append(triangles)
    ax.review_colours.append(colours)


def flush(fig):
    for ax in fig.axes:
        if getattr(ax, "review_triangles", None):
            triangles = np.concatenate(ax.review_triangles)
            colours = np.concatenate(ax.review_colours)
            elev, azim = np.radians(ax.review_camera)
            eye = np.array([np.cos(elev) * np.cos(azim), np.cos(elev) * np.sin(azim), np.sin(elev)])
            right = np.array([-np.sin(azim), np.cos(azim), 0])
            up = np.cross(eye, right)
            points = triangles @ np.stack([right, up, -eye], axis=1)
            w = int(fig.get_figwidth() * fig.dpi * ax.review_rect[2])
            h = int(fig.get_figheight() * fig.dpi * ax.review_rect[3])
            lo = points[:, :, :2].min(axis=(0, 1))
            hi = points[:, :, :2].max(axis=(0, 1))
            scale = min(w / (hi[0] - lo[0]), h / (hi[1] - lo[1])) * 0.88 * 2
            points[:, :, 0] = (points[:, :, 0] - (lo[0] + hi[0]) / 2) * scale + w
            points[:, :, 1] = h - (points[:, :, 1] - (lo[1] + hi[1]) / 2) * scale
            records = np.c_[points.reshape(-1, 9), colours].astype("<f4")
            raw = np.array([len(records)], dtype="<u4").tobytes() + records.tobytes()
            rendered = subprocess.run(
                [str(RASTER), str(w * 2), str(h * 2)], input=raw, stdout=subprocess.PIPE, check=True
            )
            bitmap = Image.open(io.BytesIO(rendered.stdout)).resize((w, h), Image.Resampling.LANCZOS)
            ax.imshow(bitmap)
            ax.set_axis_off()
            ax.review_triangles = []
            ax.review_colours = []


def setup(fig, rect, limits, elev=23, azim=-125):
    ax = fig.add_axes(rect)
    ax.set_facecolor(BG)
    ax.review_camera = (elev, azim)
    ax.review_rect = rect
    ax.review_triangles = []
    ax.review_colours = []
    ax.set_axis_off()
    return ax


def cable(ax, length=45):
    for shape, colour in [
        (box((length, 11.5, 0.14), (length / 2, 0, 0)), (0.16, 0.19, 0.22)),
        (box((6, 11.5, 0.30), (3, 0, 0)), (0.13, 0.43, 0.75)),
    ]:
        v, f = mesh_data(shape)
        solid(ax, v, f, colour)
    for i in range(22):
        v, f = mesh_data(box((4, 0.3, 0.002), (2, (i - 10.5) * 0.5, -0.151)))
        solid(ax, v, f, (0.87, 0.64, 0.21))


def tool(ax, gap=0.14, cutaway=False):
    collections = []
    for p in parts(gap):
        if cutaway and p["name"] in {"roof_rail_0", "rear_post_0", "guide_leaf_0", "guide_leaf_2"}:
            continue
        v, f = mesh_data(p["shape"])
        collections.append(solid(ax, v, f, p["color"]))
    return collections


def board(ax):
    stage = Usd.Stage.Open(str(ROOT / "third_party/raspberry_pi/zero/zero2w.usdc"))
    cache = UsdGeom.XformCache()
    for p in stage.Traverse():
        if not p.IsA(UsdGeom.Mesh) or "/Part_0010" in str(p.GetPath()):
            continue
        mesh = UsdGeom.Mesh(p)
        v = np.array(mesh.GetPointsAttr().Get())
        m = np.array(cache.GetLocalToWorldTransform(p)).T
        v = (np.c_[v, np.ones(len(v))] @ m.T)[:, :3] * 1000 + [-33.3, 0.8, -2.2]
        counts = np.array(mesh.GetFaceVertexCountsAttr().Get())
        ids = np.array(mesh.GetFaceVertexIndicesAttr().Get())
        faces = []
        off = 0
        for n in counts:
            faces.extend([[ids[off], ids[off + j], ids[off + j + 1]] for j in range(1, n - 1)])
            off += n
        colour = mesh.GetDisplayColorAttr().Get()
        rgb = list(colour[0]) if colour else [0.2, 0.35, 0.22]
        mat = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
        if mat:
            source = mat.ComputeSurfaceSource()[0]
            if source:
                value = source.GetInput("diffuseColor").Get()
                if value is not None:
                    rgb = list(value)
        solid(ax, v, np.array(faces), rgb)
    # Source-backed depth; assumed clear throat. Static reference only.
    for size, center in [
        ((2.5, 12.35, 0.35), (-1.25, 0, -0.375)),
        ((2.5, 12.35, 0.35), (-1.25, 0, 0.375)),
        ((2.5, 0.35, 1.1), (-1.25, -6, 0.0)),
        ((2.5, 0.35, 1.1), (-1.25, 6, 0)),
        ((0.35, 12.35, 1.1), (-2.675, 0, 0)),
    ]:
        v, f = mesh_data(box(size, center))
        solid(ax, v, f, (0.12, 0.14, 0.15))


def label(fig, x, y, number, title, text):
    fig.text(x, y, number, fontsize=11, color="#147b78", weight="bold")
    fig.text(x + 0.025, y, title, fontsize=13, color="#213b3c", weight="bold")
    fig.text(x + 0.025, y - 0.027, text, fontsize=10, color="#526464", linespacing=1.6, va="top")


def render(out):
    out.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["g++", "-O2", "-std=c++17", str(Path(__file__).with_name("raster.cpp")), "-o", str(RASTER)],
        check=True,
    )
    plt.rcParams.update({"font.family": "DejaVu Sans", "figure.facecolor": BG})
    fig = plt.figure(figsize=(16, 11), dpi=130)
    fig.text(0.05, 0.95, "COMPACT TOOL / CONCEPT 01", fontsize=11, color="#147b78", weight="bold")
    fig.text(0.05, 0.90, "A 30 mm custom pinch head", fontsize=29, color="#213b3c", weight="bold")
    fig.text(
        0.05,
        0.857,
        "One voice coil · fixed lower blade · flexure-guided upper finger",
        fontsize=13,
        color="#526464",
    )
    ax = setup(fig, [0.015, 0.15, 0.68, 0.69], [(1, 43), (-13, 21), (-6, 24)], 24, 135)
    tool(ax)
    cable(ax)
    label(
        fig,
        0.69,
        0.75,
        "01",
        "11.1 mm voice coil",
        "H2W candidate; envelope model.\nCurrent driver stays off the wrist.",
    )
    label(fig, 0.69, 0.63, "02", "Steel flexure guide", "Four 40 µm leaves.\nNo screw or sliding jaw guide.")
    label(
        fig,
        0.69,
        0.51,
        "03",
        "Force-sensing finger",
        "Bonded strain-gauge reserve.\nSeparate compliant contact pad.",
    )
    label(
        fig,
        0.69,
        0.39,
        "04",
        "Separate suction tip",
        "Pickup → passive fixture → pinch.\nNo bulky handoff slide on the tool.",
    )
    fig.text(0.05, 0.13, "30 × 28 × 24.7 mm", fontsize=25, color="#213b3c", weight="bold")
    fig.text(
        0.05,
        0.097,
        "Including fingers and pickup · main body 22 × 20 × 23 mm · adapter and leads excluded",
        fontsize=10,
        color="#526464",
    )
    fig.text(
        0.05,
        0.045,
        "Actual CAD render. Concept only: no force, fatigue, seal or insertion validation."
        " FR3 adapter excluded.",
        fontsize=10,
        color="#526464",
    )
    flush(fig)
    fig.savefig(out / "concept.png", facecolor=BG)
    plt.close(fig)
    fig = plt.figure(figsize=(16, 8), dpi=130)
    fig.text(0.05, 0.93, "THE SAME SCALE AS THE PI ZERO 2 W", fontsize=20, color="#213b3c", weight="bold")
    fig.text(
        0.05,
        0.875,
        "Clear end for inspection; grip the insulated body 8.5–10.5 mm behind the leading edge.",
        fontsize=12,
        color="#526464",
    )
    ax = setup(fig, [0, 0.07, 1, 0.75], [(-72, 48), (-22, 23), (-6, 25)], 35, -105)
    board(ax)
    tool(ax)
    cable(ax)
    fig.text(
        0.05,
        0.055,
        "Board: community CAD. Cable tip at mouth for scale review only."
        " No contact simulation or successful insertion implied.",
        fontsize=10,
        color="#526464",
    )
    flush(fig)
    fig.savefig(out / "board-scale.png", facecolor=BG)
    plt.close(fig)
    fig = plt.figure(figsize=(16, 8), dpi=130)
    for left, gap, title in [
        (0.0, 1.2, "OPEN / 1.20 mm pad gap"),
        (0.5, 0.14, "TOUCH / 0.14 mm body coupon"),
    ]:
        ax = setup(fig, [left, 0.14, 0.5, 0.69], [(5, 40), (-11, 18), (-4, 22)], 14, -95)
        tool(ax, gap, cutaway=True)
        cable(ax)
        fig.text(left + 0.05, 0.88, title, fontsize=18, color="#213b3c", weight="bold")
    fig.text(
        0.05,
        0.07,
        "Near-side rails and two guide leaves omitted in this cutaway."
        " Jaw movement is prescribed; loaded sensing-leaf bending is not shown.",
        fontsize=10,
        color="#526464",
    )
    flush(fig)
    fig.savefig(out / "open-closed.png", facecolor=BG)
    plt.close(fig)
    fig = plt.figure(figsize=(9, 6), dpi=85)
    writer = PillowWriter(fps=8)
    with writer.saving(fig, str(out / "jaw-preview.gif"), dpi=85):
        for gap in np.r_[np.linspace(1.2, 0.14, 9), [0.14] * 4, np.linspace(0.14, 1.2, 9), [1.2] * 4]:
            fig.clear()
            ax = setup(fig, [0, 0.05, 1, 0.85], [(5, 40), (-11, 18), (-4, 22)], 15, -115)
            tool(ax, float(gap), True)
            cable(ax)
            fig.text(0.06, 0.94, f"GEOMETRY PREVIEW  /  GAP {gap:.2f} mm", fontsize=13, color="#213b3c")
            fig.text(
                0.06,
                0.04,
                "Prescribed travel; not a physics or grasp demonstration.",
                fontsize=10,
                color="#526464",
            )
            flush(fig)
            writer.grab_frame(facecolor=BG)
    (out / "render-scope.json").write_text(
        json.dumps(
            {
                "engine": "CAD tessellation, CPU z-buffer, Matplotlib labels",
                "physics": False,
                "supplier_geometry": "Own envelopes",
                "board": "Existing community Zero CAD",
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    render(p.parse_args().output)
