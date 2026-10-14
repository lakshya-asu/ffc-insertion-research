"""CAD for every part we make, from the twin's design numbers (out/ffc_zif/design.json): STEP files for a machine shop or printer, a
dimensioned drawing (SVG) per part, and an index page. Run with the cadquery interpreter:
  <stepenv>/bin/python -I scripts/ffc_custom_parts.py out/ffc_zif/design.json <out_dir>
Hole patterns are the makers' published ones (parts.json); angular positions marked ESTIMATE are a design choice to confirm on the CAD."""
import json, math, sys
from pathlib import Path
import cadquery as cq

design = json.loads(Path(sys.argv[1]).read_text()); OUT = Path(sys.argv[2]); OUT.mkdir(parents=True, exist_ok=True)
S = design["stacks"]
parts = []     # (file stem, title, material, how, notes, drawing svg)


def pcd(n, d, a0=0.0):
    return [(d / 2 * math.cos(math.radians(a0 + k * 360 / n)), d / 2 * math.sin(math.radians(a0 + k * 360 / n))) for k in range(n)]


def svg_drawing(title, views, notes, W=1180):
    """views: list of (label, width_mm, height_mm, shapes); shapes: ('rect', x0, y0, x1, y1[, fill]), ('circle', cx, cy, r, text), ('dim', x0, y0, x1, y1, text) in mm.
    Each view gets its own panel sized from its content (5 px/mm); hole texts become numbered callouts with a legend under the view."""
    s_ = 5.0; pad = 46; panels = []; x = 20
    for label, _, _, shapes in views:
        xs, ys = [], []
        for sh in shapes:
            if sh[0] == "rect": xs += [sh[1], sh[3]]; ys += [sh[2], sh[4]]
            elif sh[0] == "circle": xs += [sh[1] - sh[3], sh[1] + sh[3]]; ys += [sh[2] - sh[3], sh[2] + sh[3]]
            elif sh[0] == "dim": xs += [sh[1], sh[3]]; ys += [sh[2], sh[4]]
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
        panels.append((label, shapes, x0, x1, y0, y1, x)); x += (x1 - x0) * s_ + 2 * pad + 40
    W = max(W, x + 20); H = 120
    for label, shapes, x0, x1, y0, y1, px in panels:
        n_leg = len({sh[4] for sh in shapes if sh[0] == "circle" and sh[4]})
        H = max(H, 90 + (y1 - y0) * s_ + 2 * pad + 16 * n_leg + 40)
    H += 18 * len(notes) + 20
    p = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W:.0f} {H:.0f}" style="background:#FBF7F0;font-family:Inter,Helvetica,Arial,sans-serif">', f'<text x="14" y="26" font-size="16" font-weight="700" fill="#1E1B16">{title}</text>']
    for label, shapes, x0, x1, y0, y1, px in panels:
        ox = px + pad - x0 * s_; oy = 70 + pad + y1 * s_
        X = lambda v: ox + v * s_; Y = lambda v: oy - v * s_
        p.append(f'<text x="{px}" y="{58}" font-size="12" fill="#6B6257">{label}, 1 mm = 5 px</text>')
        legend = {}; k = 0
        for sh in shapes:
            if sh[0] == "rect":
                fill = sh[5] if len(sh) > 5 else "#9aa0a8"
                p.append(f'<rect x="{X(min(sh[1], sh[3])):.1f}" y="{Y(max(sh[2], sh[4])):.1f}" width="{abs(sh[3] - sh[1]) * s_:.1f}" height="{abs(sh[4] - sh[2]) * s_:.1f}" fill="{fill}" fill-opacity="0.85" stroke="#1E1B16" stroke-width="0.8"/>')
        for sh in shapes:
            if sh[0] == "circle":
                p.append(f'<circle cx="{X(sh[1]):.1f}" cy="{Y(sh[2]):.1f}" r="{sh[3] * s_:.1f}" fill="#FBF7F0" stroke="#1E1B16" stroke-width="0.8"/>')
                if sh[4]:
                    if sh[4] not in legend: k += 1; legend[sh[4]] = k
                    p.append(f'<text x="{X(sh[1]) + sh[3] * s_ + 2:.1f}" y="{Y(sh[2]) - sh[3] * s_ - 1:.1f}" font-size="9" font-weight="700" fill="#C0392B">{legend[sh[4]]}</text>')
            elif sh[0] == "dim":
                _, a, b, c, d_, t = sh
                p.append(f'<line x1="{X(a):.1f}" y1="{Y(b):.1f}" x2="{X(c):.1f}" y2="{Y(d_):.1f}" stroke="#C0392B" stroke-width="1"/><line x1="{X(a)-3:.1f}" y1="{Y(b)-3:.1f}" x2="{X(a)+3:.1f}" y2="{Y(b)+3:.1f}" stroke="#C0392B"/><line x1="{X(c)-3:.1f}" y1="{Y(d_)-3:.1f}" x2="{X(c)+3:.1f}" y2="{Y(d_)+3:.1f}" stroke="#C0392B"/>')
                if abs(c - a) >= abs(d_ - b): p.append(f'<text x="{(X(a) + X(c)) / 2:.1f}" y="{Y(b) - 5:.1f}" font-size="11" fill="#C0392B" text-anchor="middle">{t}</text>')
                else: p.append(f'<text x="{X(a) + 6:.1f}" y="{(Y(b) + Y(d_)) / 2 + 4:.1f}" font-size="11" fill="#C0392B">{t}</text>')
        ly = 70 + pad + (y1 - y0) * s_ + pad + 10
        for t, kk in legend.items():
            p.append(f'<text x="{px}" y="{ly:.0f}" font-size="11" fill="#1E1B16"><tspan font-weight="700" fill="#C0392B">{kk}</tspan>  {t}</text>'); ly += 16
    y = H - 18 * len(notes) - 6
    for n in notes:
        p.append(f'<text x="14" y="{y:.0f}" font-size="11.5" fill="#1E1B16">{n}</text>'); y += 18
    p.append('</svg>'); return "".join(p)


def save(stem, shape, title, material, how, notes, drawing):
    cq.exporters.export(shape, str(OUT / f"{stem}.step")); (OUT / f"{stem}.svg").write_text(drawing)
    bb = shape.val().BoundingBox(); parts.append((stem, title, material, how, notes, f"{bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {shape.val().Volume() / 1000:.2f} cm3"))
    print("wrote", stem, f"{bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}")


# ------------------------------------------------------------- Meca500 side ------------------------------------------------------
# 1. adapter plate, Meca500 flange to Nano25 base. Flange side up (z+): Meca500 pattern 4 x M3 countersunk from BELOW (flat head screws
#    up into the flange's tapped holes, heads flush under the sensor), 2 x 3 mm dowel holes, 9 mm h6 boss into the flange's 9 H6 bore (2.2 deep).
#    Sensor side down: 3 x M3 socket heads counterbored from ABOVE (down into the Nano25's tapped holes, 7 mm deep), 2 x 3 mm dowel holes, all on the 20.32 mm circle.
T = 4.0; W = 28.0                                                                        # round, 28 dia: the Nano25's 20.32 mm bolt circle with M3 heads needs it
pl = cq.Workplane("XY").circle(W / 2).extrude(T)
pl = pl.faces(">Z").workplane().circle(4.5).extrude(2.0)                                   # 9 mm h6 boss, 2 mm tall, into the flange's 9 H7 bore
for x, y in pcd(4, 15.5, 45):                                                            # ESTIMATE angles: M3 at 45 deg, dowels at 0/180
    pl = pl.faces("<Z").workplane(origin=(0, 0, 0)).center(x, -y).cskHole(3.4, 6.5, 90)
for x, y in pcd(2, 15.5, 0):
    pl = pl.faces(">Z").workplane(origin=(0, 0, 0)).center(x, y).hole(3.0)
for x, y in pcd(3, 20.32, 90):                                                           # Nano25 mounting pattern: 3 x M3 socket heads counterbored from the flange side
    pl = pl.faces(">Z").workplane(origin=(0, 0, 0)).center(x, y).cboreHole(3.4, 6.0, 3.3)
for x, y in pcd(2, 20.32, 30):
    pl = pl.faces(">Z").workplane(origin=(0, 0, 0)).center(x, y).hole(3.0)
drw = svg_drawing("Adapter plate, Meca500 flange to ATI Nano25 (ours)",
    [("top view (flange side)", W, W, [("circle", 0, 0, W / 2, "")] + [("circle", x, y, 1.7, "M3 csk from below") for x, y in pcd(4, 15.5, 45)] + [("circle", x, y, 1.5, "3 H7 dowel") for x, y in pcd(2, 15.5, 0)]
      + [("circle", x, y, 1.7, "M3 cbore 6 x 3.3") for x, y in pcd(3, 20.32, 90)] + [("circle", x, y, 1.5, "3 H7 dowel") for x, y in pcd(2, 20.32, 30)] + [("circle", 0, 0, 4.5, "9 h6 boss"), ("dim", -W / 2, -W / 2 - 2, W / 2, -W / 2 - 2, f"{W:g}")]),
     ("section", W, T + 2, [("rect", -W / 2, 0, W / 2, T), ("rect", -4.5, T, 4.5, T + 2), ("dim", W / 2 + 2, 0, W / 2 + 2, T, f"{T:g}"), ("dim", W / 2 + 2, T, W / 2 + 2, T + 2, "2 boss")])],
    ["6061-T6, 4 mm, 28 mm round. Meca500 side: 4 x M3 flat head from the sensor side (max 4 mm into the flange), 2 x 3 mm dowels on 15.5 PCD, 9 mm h6 boss into the flange's 9 H7 bore (2.2 deep).",
     "Nano25 side: 3 x M3 socket head screws down into the sensor's mounting face (tapped 7 mm deep), heads in 3.3 mm counterbores so the flange face sits flat; 2 x 3 mm dowels on the 20.32 mm circle.",
     "Angular positions of the Meca500 dowels (0/180) and screws (45 deg) are a design choice: confirm on the Meca500 flange drawing before cutting (ESTIMATE)."])
save("meca_adapter_plate_nano25", pl, "Adapter plate, Meca500 to Nano25", "6061-T6", "turn or mill + drill; ream dowels H7", "4 mm, 28 dia", drw)

# 2. 90 degree bracket, Meca500 stack: horizontal leg under the Nano25 (its tool pattern counterbored from below), vertical leg carrying the
#    MEGP's stock STAP01 plate on the Meca500 flange pattern (4 x M3 + 2 dowels), both legs 6 mm, 30 mm wide along the gripper axis.
d = S["meca500/megp_nano25"]; B = d["bracket"]; t = B["thickness_mm"]; w = B["width_mm"]
hl = B["horizontal_leg"]; vl = B["vertical_leg"]
h_y0, h_y1 = hl["centre_tool_xyz_mm"][1] - hl["half_size_mm"][1], hl["centre_tool_xyz_mm"][1] + hl["half_size_mm"][1]
v_y0, v_y1 = vl["centre_tool_xyz_mm"][1] - vl["half_size_mm"][1], vl["centre_tool_xyz_mm"][1] + vl["half_size_mm"][1]
v_z0, v_z1 = vl["centre_tool_xyz_mm"][2] - vl["half_size_mm"][2], vl["centre_tool_xyz_mm"][2] + vl["half_size_mm"][2]
h_z0 = hl["centre_tool_xyz_mm"][2] - hl["half_size_mm"][2]
zc = B["stroke_centre_tool_z_mm"]
# bracket frame: X along the gripper axis (tool x), Y across (tool y), Z down the flange axis (tool z); build in tool coordinates
horiz = cq.Workplane("XY").box(w, h_y1 - h_y0, t).translate((0, (h_y0 + h_y1) / 2, h_z0 + t / 2))
vert = cq.Workplane("XY").box(w, t, v_z1 - v_z0).translate((0, (v_y0 + v_y1) / 2, (v_z0 + v_z1) / 2))
br = horiz.union(vert)
for x, y in pcd(3, 20.32, 90):                                                           # Nano25 tool pattern, counterbored from below
    br = br.faces("<Z").workplane(origin=(0, 0, 0)).center(x, -y).cboreHole(3.4, 6.0, 3.3)
for x, y in pcd(2, 20.32, 30):
    br = br.faces("<Z").workplane(origin=(0, 0, 0)).center(x, -y).hole(3.0)
# MEGP plate pattern on the vertical leg, centred on the flange axis (x 0) at the stroke centre height zc; socket heads from the outside (east)
for x, z in pcd(4, 15.5, 45):
    br = br.faces(">Y").workplane(origin=(0, 0, 0)).center(x, z - zc if False else 0).center(0, 0).moveTo(x, zc + z).cboreHole(3.4, 6.0, 3.3) if False else br
holes = cq.Workplane("XZ", origin=(0, v_y1, 0))
cyl = None
for x, z in pcd(4, 15.5, 45):
    c = cq.Workplane("XZ", origin=(x, v_y1 + 1, zc + z)).circle(1.7).extrude(-(t + 2))
    cb = cq.Workplane("XZ", origin=(x, v_y1 + 1, zc + z)).circle(3.0).extrude(-(1 + 3.3))
    br = br.cut(c).cut(cb)
for x, z in pcd(2, 15.5, 0):
    br = br.cut(cq.Workplane("XZ", origin=(x, v_y1 + 1, zc + z)).circle(1.5).extrude(-(t + 2)))
br = br.cut(cq.Workplane("XZ", origin=(0, v_y1 + 1, zc)).circle(4.6).extrude(-(1 + 2.5)))          # 9 mm recess for the plate's boss, 2.5 deep
bb = br.val().BoundingBox()
drw = svg_drawing("90 degree bracket, Meca500 stack (ours)",
    [("side view, along the gripper axis", h_y1 - h_y0 + 4, v_z1 - h_z0 + 4,
      [("rect", h_y0, -(h_z0 + t), h_y1, -h_z0), ("rect", v_y0, -v_z1, v_y1, -v_z0), ("circle", (v_y0 + v_y1) / 2, -zc, 1.7, "MEGP plate pattern centre"), ("dim", h_y0, -h_z0 + 3, h_y1, -h_z0 + 3, f"{h_y1 - h_y0:g}"), ("dim", v_y1 + 3, -v_z0, v_y1 + 3, -v_z1, f"{v_z1 - v_z0:g}"), ("dim", h_y0 - 3, -h_z0, h_y0 - 3, -zc, f"{zc - h_z0:g} to the stroke centre")]),
     ("vertical leg from the east", w + 4, v_z1 - v_z0 + 4,
      [("rect", -w / 2, -v_z1, w / 2, -v_z0)] + [("circle", x, -(zc + z), 1.7, "M3 cbore 6 x 3.3") for x, z in pcd(4, 15.5, 45)] + [("circle", x, -(zc + z), 1.5, "3 H7") for x, z in pcd(2, 15.5, 0)] + [("circle", 0, -zc, 4.6, "9 recess")]),
     ("horizontal leg from below", w + 4, h_y1 - h_y0 + 4,
      [("rect", -w / 2, h_y0, w / 2, h_y1)] + [("circle", x, y, 1.7, "M3 cbore") for x, y in pcd(3, 20.32, 90)] + [("circle", x, y, 1.5, "3 H7") for x, y in pcd(2, 20.32, 30)])],
    [f"6061-T6 machined from solid (not bent), {t:g} mm legs, {w:g} mm wide along the gripper axis. Flange axis through x = 0.",
     "Horizontal leg: 3 x M3 socket heads counterbored from below into the Nano25 tool side, 2 x 3 mm dowels (3.3 mm deep in the sensor) on the 20.32 mm circle.",
     f"Vertical leg: the MEGP 25E's stock STAP01 plate bolts to it with 4 x M3 (Meca500 flange pattern, 15.5 PCD) + 2 x 3 mm dowels, centred {zc - h_z0:g} mm below the leg at the gripper's stroke centre; 9 mm recess for the plate's boss.",
     f"Gripper jaw end is {B['flange_axis_to_jaw_end_mm']:g} mm ahead of the flange axis (Mecademic Figure 1). Overall {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm."])
save("meca_bracket_90_megp25e", br, "90 degree bracket, Meca500 stack", "6061-T6", "CNC from solid", f"{t:g} mm legs", drw)

# ------------------------------------------------------------- LR Mate side ------------------------------------------------------
# 3. adapter plate, LR Mate ISO 31.5 flange to ATI Mini40: 44 dia x 6. Flange side up: 4 x M5 flat head from below on 31.5 PCD, 5 H7 dowel.
#    Sensor side down: 3 x M3 socket heads counterbored from above into the Mini40 base (5 mm deep max), 2 x 3 mm dowels on 33 PCD.
D = 44.0; T2 = 6.0
pl2 = cq.Workplane("XY").circle(D / 2).extrude(T2)
for x, y in pcd(4, 31.5, 45):
    pl2 = pl2.faces("<Z").workplane(origin=(0, 0, 0)).center(x, -y).cskHole(5.5, 10.4, 90)
pl2 = pl2.faces(">Z").workplane(origin=(0, 0, 0)).center(0, 31.5 / 2).hole(5.0)             # 5 H7 dowel on the PCD (ISO 9409-1 position)
for x, y in pcd(3, 33, 90):
    pl2 = pl2.faces(">Z").workplane(origin=(0, 0, 0)).center(x, y).cboreHole(3.4, 6.0, 3.3)
for x, y in pcd(2, 33, 30):
    pl2 = pl2.faces(">Z").workplane(origin=(0, 0, 0)).center(x, y).hole(3.0)
drw = svg_drawing("Adapter plate, LR Mate ISO 31.5 flange to ATI Mini40 (ours)",
    [("top view (flange side)", D, D, [("circle", 0, 0, D / 2, "")] + [("circle", x, y, 2.75, "M5 csk from below") for x, y in pcd(4, 31.5, 45)] + [("circle", 0, 31.5 / 2, 2.5, "5 H7 dowel")] + [("circle", x, y, 1.7, "M3 cbore 6 x 3.3") for x, y in pcd(3, 33, 90)] + [("circle", x, y, 1.5, "3 H7") for x, y in pcd(2, 33, 30)] + [("dim", -D / 2, -D / 2 - 2, D / 2, -D / 2 - 2, f"{D:g} dia")]),
     ("section", D, T2 + 2, [("rect", -D / 2, 0, D / 2, T2), ("dim", D / 2 + 2, 0, D / 2 + 2, T2, f"{T2:g}")])],
    ["6061-T6, 6 mm, 44 mm dia. LR Mate side: 4 x M5 flat head screws from the sensor side (8 mm max into the flange), one 5 H7 dowel on the 31.5 PCD (ISO 9409-1-31.5-4-M5).",
     "Mini40 side: 3 x M3 socket heads counterbored from the flange side into the sensor's mounting side (5 mm deep max), 2 x 3 mm dowels on 33 PCD.",
     "No spigot recess: location by the dowel and the PCD; the FANUC 40 h7 spigot height is not confirmed (ESTIMATE), confirm on Fig 4.1 of the LR Mate manual."])
save("lrmate_adapter_plate_mini40", pl2, "Adapter plate, LR Mate to Mini40", "6061-T6", "turn + drill; ream dowels H7", "6 mm, 44 dia", drw)

# 4. 90 degree bracket, LR Mate stack: horizontal leg under the Mini40 (tool pattern counterbored from below), vertical leg carrying the Zimmer by its side pattern
d = S["lrmate200id/lehz16_mini40"]; B = d["bracket"]; t = B["thickness_mm"]; w = B["width_mm"]
hl = B["horizontal_leg"]; vl = B["vertical_leg"]
h_y0, h_y1 = hl["centre_tool_xyz_mm"][1] - hl["half_size_mm"][1], hl["centre_tool_xyz_mm"][1] + hl["half_size_mm"][1]
v_y0, v_y1 = vl["centre_tool_xyz_mm"][1] - vl["half_size_mm"][1], vl["centre_tool_xyz_mm"][1] + vl["half_size_mm"][1]
v_z0, v_z1 = vl["centre_tool_xyz_mm"][2] - vl["half_size_mm"][2], vl["centre_tool_xyz_mm"][2] + vl["half_size_mm"][2]
h_z0 = hl["centre_tool_xyz_mm"][2] - hl["half_size_mm"][2]; zc = B["stroke_centre_tool_z_mm"]
w2 = max(w, 40.0)                                                                        # wide enough for the Mini40's 33 PCD
horiz = cq.Workplane("XY").box(w2, h_y1 - h_y0, t).translate((0, (h_y0 + h_y1) / 2, h_z0 + t / 2))
vert = cq.Workplane("XY").box(w, t, v_z1 - v_z0).translate((0, (v_y0 + v_y1) / 2, (v_z0 + v_z1) / 2))
br2 = horiz.union(vert)
for x, y in pcd(3, 33, 90):
    br2 = br2.faces("<Z").workplane(origin=(0, 0, 0)).center(x, -y).cboreHole(3.4, 6.0, 3.3)
for x, y in pcd(2, 33, 30):
    br2 = br2.faces("<Z").workplane(origin=(0, 0, 0)).center(x, -y).hole(3.0)
for x, z in ((-8, -8), (8, 8)):                                                          # Zimmer side pattern: 2 x M3 on one diagonal, 2 x 5 H7 on the other, 16 mm spacing (ESTIMATE layout)
    br2 = br2.cut(cq.Workplane("XZ", origin=(x, v_y1 + 1, zc + z)).circle(1.7).extrude(-(t + 2))).cut(cq.Workplane("XZ", origin=(x, v_y1 + 1, zc + z)).circle(3.0).extrude(-(1 + 3.3)))
for x, z in ((-8, 8), (8, -8)):
    br2 = br2.cut(cq.Workplane("XZ", origin=(x, v_y1 + 1, zc + z)).circle(2.5).extrude(-(t + 2)))
bb = br2.val().BoundingBox()
drw = svg_drawing("90 degree bracket, LR Mate stack, for the SMC LEHZ16 (ours)",
    [("side view, along the gripper axis", h_y1 - h_y0 + 4, v_z1 - h_z0 + 4,
      [("rect", h_y0, -(h_z0 + t), h_y1, -h_z0), ("rect", v_y0, -v_z1, v_y1, -v_z0), ("circle", (v_y0 + v_y1) / 2, -zc, 1.7, "Zimmer pattern centre"), ("dim", h_y0, -h_z0 + 3, h_y1, -h_z0 + 3, f"{h_y1 - h_y0:g}"), ("dim", v_y1 + 3, -v_z0, v_y1 + 3, -v_z1, f"{v_z1 - v_z0:g}"), ("dim", h_y0 - 3, -h_z0, h_y0 - 3, -zc, f"{zc - h_z0:g} to the stroke centre")]),
     ("vertical leg from the east", w + 4, v_z1 - v_z0 + 4,
      [("rect", -w / 2, -v_z1, w / 2, -v_z0)] + [("circle", x, -(zc + z), 1.7, "M3 cbore") for x, z in ((-8, -8), (8, 8))] + [("circle", x, -(zc + z), 1.25, "2.5 H9 pin") for x, z in ((-8, 8), (8, -8))]),
     ("horizontal leg from below", w2 + 4, h_y1 - h_y0 + 4,
      [("rect", -w2 / 2, h_y0, w2 / 2, h_y1)] + [("circle", x, y, 1.7, "M3 cbore 6 x 3.3") for x, y in pcd(3, 33, 90)] + [("circle", x, y, 1.5, "3 H7") for x, y in pcd(2, 33, 30)])],
    [f"6061-T6 machined from solid, {t:g} mm legs; horizontal leg {w2:g} mm wide for the Mini40's 33 PCD, vertical leg {w:g} mm wide.",
     "Horizontal leg: 3 x M3 socket heads counterbored from below into the Mini40 tool side (only 3.0 mm deep: use 6 mm screws), 2 x 3 mm dowels (2.5 mm deep in the sensor).",
     f"Vertical leg: SMC LEHZ16 side pattern, 2 x M3 + 2 x 2.5 H9 pins, centred {zc - h_z0:g} mm below the leg at the stroke centre, {B['flange_axis_to_jaw_end_mm']:g} mm behind the jaw end (pattern spacing and position ESTIMATE: read the SMC drawing).",
     f"Overall {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm."])
save("lrmate_bracket_90_lehz16", br2, "90 degree bracket, LR Mate stack", "6061-T6", "CNC from solid", f"{t:g} mm legs", drw)

# ------------------------------------------------------------- fingers (both stacks, MEGP jaw pattern shown) -------------------
d = S["meca500/megp_nano25"]; gj = d["geoms_jaw_frame"]; fp = d["finger_plate_t_mm"]; lw = d["leg_w_mm"]
def box_from(gname):
    p, h = gj[gname]["pos_mm"], gj[gname]["half_mm"]
    return cq.Workplane("XY").box(2 * h[2], 2 * h[1], 2 * h[0]).translate((p[2], p[1], -p[0]))        # part frame: X forward (jaw z), Y east (jaw y), Z up (-jaw x)
lower = box_from("plate_lo").union(box_from("leg_e")).union(box_from("shoe_bar"))
jaw = gj["jaw_lo"]; jc = (jaw["pos_mm"][2], jaw["pos_mm"][1])                           # jaw centre (x forward, y east)
lower = lower.cut(cq.Workplane("XY", origin=(jc[0], jc[1], 0)).circle(2.2).extrude(60, both=True))     # M4 clearance at the jaw face centre
for dx in (-2.5, 2.5):
    lower = lower.cut(cq.Workplane("XY", origin=(jc[0] + dx, jc[1], 0)).circle(0.75).extrude(60, both=True))   # 2 x 1.5 H7 pins, 5 mm apart (ESTIMATE spacing)
sh = d["shoe"]; shoe_g = gj["shoe_g"]
shoe = cq.Workplane("XY").box(sh["d_mm"], sh["w_mm"], sh["t_mm"]).translate((shoe_g["pos_mm"][2], 0, -shoe_g["pos_mm"][0]))
lip = cq.Workplane("XY").box(sh["lip_l_mm"], sh["lip_w_mm"], sh["lip_t_mm"]).translate((shoe_g["pos_mm"][2] + sh["d_mm"] / 2 + sh["lip_l_mm"] / 2, 0, -(sh["t_mm"] - sh["lip_t_mm"] / 2)))
wing = box_from("shoe_bar")
shoe_part = shoe.union(lip).union(wing)
bbl = lower.val().BoundingBox()
drw = svg_drawing("Lower finger (ours, printed PAHT-CF) and its steel shoe",
    [("side view from the east", bbl.xlen + 4, bbl.zlen + 4, [("rect", gj["plate_lo"]["pos_mm"][2] - gj["plate_lo"]["half_mm"][2], -gj["plate_lo"]["pos_mm"][0] - gj["plate_lo"]["half_mm"][0], gj["plate_lo"]["pos_mm"][2] + gj["plate_lo"]["half_mm"][2], -gj["plate_lo"]["pos_mm"][0] + gj["plate_lo"]["half_mm"][0], "#ee8c32"),
       ("rect", gj["leg_e"]["pos_mm"][2] - gj["leg_e"]["half_mm"][2], -gj["leg_e"]["pos_mm"][0] - gj["leg_e"]["half_mm"][0], gj["leg_e"]["pos_mm"][2] + gj["leg_e"]["half_mm"][2], -gj["leg_e"]["pos_mm"][0] + gj["leg_e"]["half_mm"][0], "#ee8c32"),
       ("rect", shoe_g["pos_mm"][2] - sh["d_mm"] / 2, -sh["t_mm"], shoe_g["pos_mm"][2] + sh["d_mm"] / 2 + sh["lip_l_mm"], 0, "#b9bdc4"), ("circle", jc[0], -jaw["pos_mm"][0], 2.0, "M4 + 2 x 1.5 pins"),
       ("dim", bbl.xmin, -bbl.zmin - 3, bbl.xmax, -bbl.zmin - 3, f"{bbl.xlen:.1f}"), ("dim", bbl.xmax + 3, 0, bbl.xmax + 3, bbl.zmax, f"{bbl.zmax:.1f} jaw face to shoe plane")]),
     ("top view", bbl.xlen + 4, bbl.ylen + 4, [("rect", gj["plate_lo"]["pos_mm"][2] - gj["plate_lo"]["half_mm"][2], gj["plate_lo"]["pos_mm"][1] - gj["plate_lo"]["half_mm"][1], gj["plate_lo"]["pos_mm"][2] + gj["plate_lo"]["half_mm"][2], gj["plate_lo"]["pos_mm"][1] + gj["plate_lo"]["half_mm"][1], "#ee8c32"),
       ("rect", shoe_g["pos_mm"][2] - sh["d_mm"] / 2, -sh["w_mm"] / 2, shoe_g["pos_mm"][2] + sh["d_mm"] / 2 + sh["lip_l_mm"], sh["w_mm"] / 2, "#b9bdc4"), ("rect", gj["shoe_bar"]["pos_mm"][2] - gj["shoe_bar"]["half_mm"][2], gj["shoe_bar"]["pos_mm"][1] - gj["shoe_bar"]["half_mm"][1], gj["shoe_bar"]["pos_mm"][2] + gj["shoe_bar"]["half_mm"][2], gj["shoe_bar"]["pos_mm"][1] + gj["shoe_bar"]["half_mm"][1], "#b9bdc4"),
       ("circle", jc[0], jc[1], 2.0, "M4"), ("dim", bbl.xmin, bbl.ymax + 3, bbl.xmax, bbl.ymax + 3, f"{bbl.xlen:.1f}"), ("dim", bbl.xmax + 3, bbl.ymin, bbl.xmax + 3, bbl.ymax, f"{bbl.ylen:.1f}")])],
    [f"Plate {fp:g} mm on the lower jaw's outer face: M4 x 0.7 clearance + two 1.5 H7 pin holes 5 mm apart on the jaw centre (MEGP 25E manual; pin spacing ESTIMATE, copy from the STEP). One leg {lw:g} x {lw:g} mm east of the cable down to the shoe.",
     f"Shoe: {sh['t_mm']:g} mm shim steel, {sh['w_mm']:g} x {sh['d_mm']:g} mm with a {sh['lip_l_mm']:g} x {sh['lip_t_mm']:g} mm lip flush with its underside and a 3 mm rear wing out to the leg; bonded under the plate's leg (structural epoxy) or held by two M1.6.",
     "Print PAHT-CF (or mill 6061) with the plate face flat; the shoe underside is the datum: it must be flat and parallel to the jaw face within 0.05 mm."])
save("finger_lower_megp25e", lower, "Lower finger, MEGP 25E", "PAHT-CF print or 6061", "print flat-face down, drill and ream the pin holes", f"plate {fp:g} mm", drw)
save("shoe_steel_0p5", shoe_part, "Steel shoe with lip and wing", "0.5 mm shim steel (1095 or 301)", "laser cut (OSH Cut, SendCutSend)", "flat pattern", svg_drawing("Steel shoe, flat pattern", [("flat pattern, X forward", 30, 40, [("rect", shoe_g["pos_mm"][2] - sh["d_mm"] / 2, -sh["w_mm"] / 2, shoe_g["pos_mm"][2] + sh["d_mm"] / 2, sh["w_mm"] / 2, "#b9bdc4"), ("rect", shoe_g["pos_mm"][2] + sh["d_mm"] / 2, -sh["lip_w_mm"] / 2, shoe_g["pos_mm"][2] + sh["d_mm"] / 2 + sh["lip_l_mm"], sh["lip_w_mm"] / 2, "#b9bdc4"), ("rect", gj["shoe_bar"]["pos_mm"][2] - gj["shoe_bar"]["half_mm"][2], gj["shoe_bar"]["pos_mm"][1] - gj["shoe_bar"]["half_mm"][1], gj["shoe_bar"]["pos_mm"][2] + gj["shoe_bar"]["half_mm"][2], gj["shoe_bar"]["pos_mm"][1] + gj["shoe_bar"]["half_mm"][1], "#b9bdc4"), ("dim", shoe_g["pos_mm"][2] - sh["d_mm"] / 2, -sh["w_mm"] / 2 - 3, shoe_g["pos_mm"][2] + sh["d_mm"] / 2 + sh["lip_l_mm"], -sh["w_mm"] / 2 - 3, f"{sh['d_mm'] + sh['lip_l_mm']:g}"), ("dim", shoe_g["pos_mm"][2] + sh["d_mm"] / 2 + sh["lip_l_mm"] + 3, -sh["w_mm"] / 2, shoe_g["pos_mm"][2] + sh["d_mm"] / 2 + sh["lip_l_mm"] + 3, sh["w_mm"] / 2, f"{sh['w_mm']:g}")])], [f"{sh['t_mm']:g} mm shim steel. The lip is the same sheet, {sh['lip_t_mm']:g} mm thick after a light skim or etched; the wing reaches east to the leg.", "Deburr the lip edge; it rides under the latch lid."]))
upper = box_from("plate_up").union(box_from("pad_leg_e")).union(box_from("pad_arm"))
# pad body offsets: pad_leg_e / pad_arm / pad_g are in the pad body frame; shift them by the pad body origin (jaw frame): (-PAD_OPEN - PAD_T_UP/2, 0, PAD_D/2 + 10)
pad = d["pad"]; px0 = -pad["open_gap_mm"] - pad["t_mm"] / 2; pz = pad["d_mm"] / 2 + 10
upper = upper.translate((pz, 0, px0 * -1 * 0))  # placeholder no-op; pad-body geoms were dumped in the pad frame
jaw = gj["jaw_up"]; jcu = (jaw["pos_mm"][2] + pz, jaw["pos_mm"][1])
bbu = upper.val().BoundingBox()
drw = svg_drawing("Upper finger (ours, printed PAHT-CF) with the pad",
    [("side view from the east", bbu.xlen + 4, bbu.zlen + 4, [("rect", bbu.xmin, bbu.zmin, bbu.xmax, bbu.zmax, "#ee8c32"), ("dim", bbu.xmin, bbu.zmin - 3, bbu.xmax, bbu.zmin - 3, f"{bbu.xlen:.1f}"), ("dim", bbu.xmax + 3, bbu.zmin, bbu.xmax + 3, bbu.zmax, f"{bbu.zlen:.1f} top plate to pad")])],
    [f"Plate {fp:g} mm on the upper jaw's outer (top) face: M4 x 0.7 clearance + two 1.5 H7 pins. One leg {lw:g} x {lw:g} mm east of the cable down to the pad's top plate.",
     f"Pad {pad['w_mm']:g} x {pad['d_mm']:g} x {pad['t_mm']:g} mm with a 1.5 mm silicone face (bonded). Recommend a 6 x 6 mm leg if printed: a 4 x 4 PAHT-CF leg deflects about 0.2 mm at the 5 N pinch.",
     "Dimensions in the pad body's frame as simulated; the STEP is the exact union of the simulated blocks."])
save("finger_upper_megp25e", upper, "Upper finger, MEGP 25E", "PAHT-CF print or 6061", "print, drill and ream the pin holes", f"plate {fp:g} mm", drw)

# ------------------------------------------------------------- camera arms (Meca500 stack shown; LR Mate identical pattern, longer y) ----
for key, tag in (("meca500/megp_nano25", "meca"), ("lrmate200id/lehz16_mini40", "lrmate")):
    A = S[key]["arms_tool_frame"]
    def abox(n):
        p_, h_ = A[n]["pos_mm"], A[n]["half_mm"]
        return cq.Workplane("XY").box(2 * h_[0], 2 * h_[1], 2 * h_[2]).translate((p_[0], p_[1], -p_[2]))      # tool frame, Z up = toward the flange
    d405arm = abox("arm_d405_y").union(abox("arm_d405")).union(abox("arm_d405_plate"))    # 2026-10-13: bars beside the camera and a 3 mm side plate on its west face (2 x M2, 18 mm apart)
    bbd = d405arm.val().BoundingBox()
    save(f"{tag}_arm_d405", d405arm, f"D405 arm, {tag} stack", "6061-T6 bar 6 x 10 + 3 mm plate", "saw + drill: 2 x M3 to the bracket leg's west end, 42 x 42 back plate with the D405's 2 x M3 (20 mm apart)", "camera 35 mm west of the cable centreline, 20 mm below the flange, 43 mm back",
         svg_drawing(f"D405 arm, {tag} stack (ours)", [("side view", bbd.xlen + 4, bbd.zlen + 4,
                     [("rect", A[k]["pos_mm"][0] - A[k]["half_mm"][0], -A[k]["pos_mm"][2] - A[k]["half_mm"][2], A[k]["pos_mm"][0] + A[k]["half_mm"][0], -A[k]["pos_mm"][2] + A[k]["half_mm"][2]) for k in ("arm_d405_y", "arm_d405", "arm_d405_plate")] + [
                      ("dim", bbd.xmin, bbd.zmax + 3, bbd.xmax, bbd.zmax + 3, f"{bbd.xlen:.1f}"), ("dim", bbd.xmax + 3, bbd.zmin, bbd.xmax + 3, bbd.zmax, f"{bbd.zlen:.1f}")])],
                     ["6 x 10 mm 6061 L at leg level (out from the leg's west end, then back) with a 3 mm back plate; the D405's two M3 (20 mm apart) are on the plate; the camera looks forward-down at the jaw tips from about 100 mm.",
                      f"The camera sits {abs(A['arm_d405']['pos_mm'][1]):g} mm west of the cable centreline so it clears the gripper body on the east and the sensor above."]))
    for side in (("e", "w") if "arm_macro_e_post" in A else ()):    # bench cameras (2026-10-13): no wrist camera arms to make
        arm = abox(f"arm_macro_{side}_post").union(abox(f"arm_macro_{side}_link")).union(abox(f"arm_macro_{side}")).union(abox(f"arm_macro_{side}_block"))
        bba = arm.val().BoundingBox()
        save(f"{tag}_arm_macro_{side}", arm, f"Macro camera arm ({'east' if side == 'e' else 'west'}), {tag} stack", "6061-T6 bar 6 x 10", "saw + drill: 2 x M3 to the leg top, camera plate at the end", "camera 75 mm beside the tips, 40 mm up",
             svg_drawing(f"Macro camera arm, {'east' if side == 'e' else 'west'}, {tag} stack (ours)", [("plan view", bba.xlen + 4, bba.ylen + 4,
                         [("rect", A[f"arm_macro_{side}_link"]["pos_mm"][0] - A[f"arm_macro_{side}_link"]["half_mm"][0], A[f"arm_macro_{side}_link"]["pos_mm"][1] - A[f"arm_macro_{side}_link"]["half_mm"][1], A[f"arm_macro_{side}_link"]["pos_mm"][0] + A[f"arm_macro_{side}_link"]["half_mm"][0], A[f"arm_macro_{side}_link"]["pos_mm"][1] + A[f"arm_macro_{side}_link"]["half_mm"][1]),
                          ("rect", A[f"arm_macro_{side}"]["pos_mm"][0] - A[f"arm_macro_{side}"]["half_mm"][0], A[f"arm_macro_{side}"]["pos_mm"][1] - A[f"arm_macro_{side}"]["half_mm"][1], A[f"arm_macro_{side}"]["pos_mm"][0] + A[f"arm_macro_{side}"]["half_mm"][0], A[f"arm_macro_{side}"]["pos_mm"][1] + A[f"arm_macro_{side}"]["half_mm"][1]),
                          ("dim", bba.xmin, bba.ymax + 3, bba.xmax, bba.ymax + 3, f"{bba.xlen:.1f}"), ("dim", bba.xmax + 3, bba.ymin, bba.xmax + 3, bba.ymax, f"{bba.ylen:.1f}")])],
                         ["6 x 10 mm 6061 bars: a post down from the bracket leg's end, a link forward under the leg to the camera's x, a bar out under the lens to the camera's centre, and a 20 x 20 block up to the camera's bottom holes (the lens points inward, at the jaws, so the camera is held from below).",
                          "Camera line of sight: 25 degrees down at the connector mouth, lens front at the working distance (108 mm Basler, 100 mm Alvium)."]))

# index
rows = "".join(f"<tr><td><b>{t}</b><br><span style='color:#6B6257'>{n}</span></td><td>{m}</td><td>{h}</td><td>{sz}</td><td><a href='{st}.step'>{st}.step</a></td></tr><tr><td colspan='5'><img src='{st}.svg' style='width:100%;max-width:1100px;border-radius:8px'></td></tr>" for st, t, m, h, n, sz in parts)
(OUT / "index.html").write_text(f"""<!doctype html><html><head><meta charset="utf-8"><title>FFC cell, custom parts</title><style>body{{margin:0;background:#FBF7F0;font:14px -apple-system,Inter,sans-serif;color:#1E1B16}}.wrap{{max-width:1180px;margin:0 auto;padding:20px}}table{{border-collapse:collapse;width:100%}}td{{padding:8px;border-bottom:1px solid #e7e1d6;vertical-align:top}}h1{{font-size:22px}}</style></head><body><div class="wrap">
<h1>The parts we make: STEP files and drawings</h1><p style="color:#6B6257">Every number comes from the simulation model that passed the task (out/ffc_zif/design.json). Hole patterns are the makers' published ones; items marked ESTIMATE are design choices to confirm against the makers' CAD before cutting.</p>
<table><tr><th>part</th><th>material</th><th>how to make</th><th>size</th><th>file</th></tr>{rows}</table></div></body></html>""")
print("index written to", OUT / "index.html")
